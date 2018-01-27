/*--------------------------------------------------------------------
  (C) Copyright 2015-2015 Barcelona Supercomputing Center
                          Centro Nacional de Supercomputacion
  
  This file is part of Mercurium C/C++ source-to-source compiler.
  
  See AUTHORS file in the top level directory for information
  regarding developers and contributors.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.
  
  Mercurium C/C++ source-to-source compiler is distributed in the hope
  that it will be useful, but WITHOUT ANY WARRANTY; without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the GNU Lesser General Public License for more
  details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with Mercurium C/C++ source-to-source compiler; if
  not, write to the Free Software Foundation, Inc., 675 Mass Ave,
  Cambridge, MA 02139, USA.
--------------------------------------------------------------------*/


#include "tl-nanos6-task-properties.hpp"
#include "tl-nanos6-lower.hpp"
#include "tl-nanos6-support.hpp"
#include "tl-nanos6-fortran-support.hpp"
#include "tl-nanos6-interface.hpp"

#include "tl-nodecl-visitor.hpp"
#include "tl-nodecl-utils.hpp"
#include "tl-nodecl-utils-fortran.hpp"
#include "tl-symbol-utils.hpp"
#include "tl-counters.hpp"

#include "codegen-phase.hpp"

#include "tl-lowering-utils.hpp"
#include "tl-atomics.hpp"

#include "cxx-typeutils.h"
#include "cxx-cexpr.h"
#include "cxx-diagnostic.h"

#include "fortran03-mangling.h"
#include "fortran03-typeutils.h"
#include "fortran03-typeenviron.h"
#include "fortran03-intrinsics.h"
#include "fortran03-scope.h"

#include <algorithm>
#include <set>

namespace TL { namespace Nanos6 {


    TaskProperties::TaskProperties(
            const Nodecl::OpenMP::Task& node,
            LoweringPhase* lowering_phase,
            Lower* lower)
        : _env(node.get_environment()), _phase(lowering_phase), _lower_visitor(lower), _num_reductions(0)
     {
        TL::Counter &counter = TL::CounterManager::get_counter("nanos6-task");
        _nanos6_task_counter = (int) counter;
        counter++;

        _locus_of_task_creation = node.get_locus();

        if (_env.locus_of_task_declaration)
            _locus_of_task_declaration = _env.locus_of_task_declaration;
        else
            _locus_of_task_declaration = node.get_locus();

        _related_function = Nodecl::Utils::get_enclosing_function(node);
        _task_body = node.get_statements();

        if (_env.is_taskloop)
        {
            ERROR_CONDITION(!_task_body.is<Nodecl::List>(), "Unexpected node\n", 0);
            Nodecl::NodeclBase stmt = _task_body.as<Nodecl::List>().front();
            ERROR_CONDITION(!stmt.is<Nodecl::Context>(), "Unexpected node\n", 0);
            stmt = stmt.as<Nodecl::Context>().get_in_context().as<Nodecl::List>().front();
            ERROR_CONDITION(!stmt.is<Nodecl::ForStatement>(), "Unexpected node\n", 0);

            TL::ForStatement for_stmt(stmt.as<Nodecl::ForStatement>());
            _taskloop_info.lower_bound = for_stmt.get_lower_bound();
            _taskloop_info.upper_bound =
                Nodecl::Add::make(
                        for_stmt.get_upper_bound(),
                        const_value_to_nodecl(const_value_get_signed_int(1)),
                        for_stmt.get_upper_bound().get_type());
            _taskloop_info.step = for_stmt.get_step();
            _taskloop_info.chunksize = _env.chunksize;
        }
    }

    TaskProperties::TaskProperties(const Nodecl::OmpSs::Release& node, LoweringPhase* lowering_phase, Lower* lower) :
        _env(node.get_environment()), _phase(lowering_phase), _lower_visitor(lower), _num_reductions(0), _nanos6_task_counter(-1)
    {
        _related_function = Nodecl::Utils::get_enclosing_function(node);
    }

    std::string TaskProperties::get_new_name(const std::string& prefix) const
    {
        std::stringstream ss;

        ss << prefix << "_";

        if (IS_FORTRAN_LANGUAGE)
            ss << _related_function.get_name() << "_";

        ss << _nanos6_task_counter;
        return ss.str();
    }

    namespace
    {
        //! Given a name (parameter) and a list of symbols (non-static data
        //! member), this functor constructs a Nodecl::Symbol if there is a
        //! symbol whose name is exactly the same as the parameter. Otherwise, it
        //! emits an error.
        struct GetField
        {
            const TL::ObjectList<TL::Symbol>& fields;

            GetField(const TL::ObjectList<TL::Symbol>& fields_) : fields(fields_) {}


            Nodecl::NodeclBase operator()(const std::string& name) const
            {
                TL::ObjectList<TL::Symbol> l;
                ERROR_CONDITION( ( l = fields.find<std::string>(&TL::Symbol::get_name, name)).empty(),
                        "Field '%s' not found", name.c_str());
                return l[0].make_nodecl(/* set_ref_type */ true);
            }
        };
    }

    void TaskProperties::create_task_info_regular_function(
        TL::Symbol task_info_struct,
        const std::string &task_info_name,
        /* out */
        TL::Symbol &task_info,
        TL::Symbol &task_invocation_info,
        Nodecl::NodeclBase &local_init)
    {
        // task info goes to the global scope
        task_info = TL::Scope::get_global_scope().new_symbol(task_info_name);
        task_info.get_internal_symbol()->kind = SK_VARIABLE;
        symbol_entity_specs_set_is_user_declared(
            task_info.get_internal_symbol(), 1);
        task_info.set_type(task_info_struct.get_user_defined_type());
        symbol_entity_specs_set_is_static(task_info.get_internal_symbol(), 1);

        // Task invocation info
        create_task_invocation_info(task_info, task_invocation_info);

        // Add required declarations to the tree
        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
        {
            if (IS_CXX_LANGUAGE)
            {
                Nodecl::Utils::prepend_to_enclosing_top_level_location(
                    _task_body,
                    Nodecl::CxxDef::make(Nodecl::NodeclBase::null(),
                                         task_info));
                Nodecl::Utils::prepend_to_enclosing_top_level_location(
                    _task_body,
                    Nodecl::CxxDef::make(Nodecl::NodeclBase::null(),
                                         task_invocation_info));
            }

            Nodecl::Utils::prepend_to_enclosing_top_level_location(
                _task_body, Nodecl::ObjectInit::make(task_info));
            Nodecl::Utils::prepend_to_enclosing_top_level_location(
                _task_body, Nodecl::ObjectInit::make(task_invocation_info));
        }
        else if (IS_FORTRAN_LANGUAGE)
        {
            _phase->get_extra_c_code().append(
                Nodecl::ObjectInit::make(task_info));
            _phase->get_extra_c_code().append(
                Nodecl::ObjectInit::make(task_invocation_info));
        }
        else
        {
            internal_error("Code unreachable", 0);
        }
    }

    void TaskProperties::create_task_info_nondependent_function(
        TL::Symbol task_info_struct,
        const std::string &task_info_name,
        /* out */
        TL::Symbol &task_info,
        TL::Symbol &task_invocation_info,
        Nodecl::NodeclBase &local_init)
    {
        if (!_related_function.is_member())
            return create_task_info_regular_function(task_info_struct,
                                                     task_info_name,
                                                     task_info,
                                                     task_invocation_info,
                                                     local_init);

        // Member
        ERROR_CONDITION(!IS_CXX_LANGUAGE, "This is only for C++", 0);

        // task_info is a static member of the class
        TL::Type class_type = _related_function.get_class_type();
        TL::Scope class_scope
            = ::class_type_get_inner_context(class_type.get_internal_type());

        task_info = class_scope.new_symbol(task_info_name);
        task_info.get_internal_symbol()->kind = SK_VARIABLE;
        symbol_entity_specs_set_is_user_declared(
            task_info.get_internal_symbol(), 1);
        task_info.set_type(task_info_struct.get_user_defined_type());
        symbol_entity_specs_set_is_member(task_info.get_internal_symbol(), 1);
        symbol_entity_specs_set_class_type(task_info.get_internal_symbol(),
                                           class_type.get_internal_type());
        symbol_entity_specs_set_is_static(task_info.get_internal_symbol(), 1);
        symbol_entity_specs_set_access(task_info.get_internal_symbol(),
                                       AS_PUBLIC);
        class_type_add_member(class_type.get_internal_type(),
                              task_info.get_internal_symbol(),
                              task_info.get_internal_symbol()->decl_context,
                              /* is_definition */ 0);

        set_is_dependent_type(class_type.get_internal_type(),
                              _related_function.get_class_type().is_dependent());

        // Task invocation info
        create_task_invocation_info(task_info, task_invocation_info);

        Nodecl::Utils::prepend_to_enclosing_top_level_location(
                _task_body,
                Nodecl::List::make(
                    Nodecl::CxxDef::make(Nodecl::NodeclBase::null(), task_invocation_info),
                    Nodecl::ObjectInit::make(task_invocation_info)));

        Nodecl::Utils::append_to_top_level_nodecl(
                 Nodecl::List::make(
                     Nodecl::CxxDef::make(Nodecl::Context::make(nodecl_null(), TL::Scope::get_global_scope()), task_info),
                     Nodecl::ObjectInit::make(task_info)));
    }

    void TaskProperties::create_task_info_dependent_function(
        TL::Symbol task_info_struct,
        const std::string &task_info_name,
        /* out */
        TL::Symbol &task_info,
        TL::Symbol &task_invocation_info,
        Nodecl::NodeclBase &local_init)
    {
        ERROR_CONDITION(!IS_CXX_LANGUAGE, "This is only for C++", 0);

        TL::Scope scope_of_template_class;
        if (!_related_function.is_member())
        {

            // We want task_info symbol be in the anonymous namespace of the
            // global
            // scope, so make sure it has been created
            Source src;
            src << "namespace { }";
            src.parse_global(TL::Scope::get_global_scope());
            //

            TL::Symbol anonymous_namespace
                = TL::Scope::get_global_scope().get_symbol_from_name(
                    "(unnamed)");
            ERROR_CONDITION(!anonymous_namespace.is_valid(),
                            "Missing unnamed namespace",
                            0);
            scope_of_template_class = anonymous_namespace.get_internal_symbol()
                                          ->related_decl_context;
        }
        else
        {
            scope_of_template_class = ::class_type_get_inner_context(
                _related_function.get_class_type().get_internal_type());
        }

        std::string task_info_tpl_name = get_new_name("task_info_tpl");

        template_parameter_list_t *tpl
            = template_specialized_type_get_template_parameters(
                _related_function.get_type().get_internal_type());

        TL::Symbol new_class_symbol
            = SymbolUtils::new_class_template(task_info_tpl_name,
                                              tpl,
                                              scope_of_template_class,
                                              _locus_of_task_creation);

        if (_related_function.is_member())
        {
            type_t *current_class
                = _related_function.get_class_type().get_internal_type();
            symbol_entity_specs_set_is_member(
                new_class_symbol.get_internal_symbol(), 1);
            symbol_entity_specs_set_class_type(
                new_class_symbol.get_internal_symbol(), current_class);
            symbol_entity_specs_set_is_defined_inside_class_specifier(
                new_class_symbol.get_internal_symbol(), 1);
            symbol_entity_specs_set_access(
                new_class_symbol.get_internal_symbol(), AS_PUBLIC);
            class_type_add_member(
                current_class,
                new_class_symbol.get_internal_symbol(),
                new_class_symbol.get_internal_symbol()->decl_context,
                /* is_definition */ 1);
            class_type_set_enclosing_class_type(
                new_class_symbol.get_type().get_internal_type(), current_class);
        }

        TL::Scope class_scope(class_type_get_inner_context(
            new_class_symbol.get_type().get_internal_type()));


        // Now add the field
        task_info = class_scope.new_symbol(task_info_name);
        task_info.get_internal_symbol()->kind = SK_VARIABLE;
        symbol_entity_specs_set_is_user_declared(
            task_info.get_internal_symbol(), 1);
        task_info.set_type(task_info_struct.get_user_defined_type());
        symbol_entity_specs_set_is_member(task_info.get_internal_symbol(), 1);
        symbol_entity_specs_set_class_type(
            task_info.get_internal_symbol(),
            new_class_symbol.get_user_defined_type().get_internal_type());
        symbol_entity_specs_set_is_static(task_info.get_internal_symbol(), 1);
        symbol_entity_specs_set_access(task_info.get_internal_symbol(),
                                       AS_PUBLIC);
        class_type_add_member(new_class_symbol.get_type().get_internal_type(),
                              task_info.get_internal_symbol(),
                              task_info.get_internal_symbol()->decl_context,
                              /* is_definition */ 0);

        // Finish the template class
        nodecl_t nodecl_output = nodecl_null();
        finish_class_type(
            new_class_symbol.get_type().get_internal_type(),
            ::get_user_defined_type(new_class_symbol.get_internal_symbol()),
            new_class_symbol.get_scope().get_decl_context(),
            _locus_of_task_creation,
            &nodecl_output);
        set_is_complete_type(new_class_symbol.get_type().get_internal_type(),
                             /* is_complete */ 1);
        set_is_complete_type(
            get_actual_class_type(
                new_class_symbol.get_type().get_internal_type()),
            /* is_complete */ 1);

        // Task invocation
        create_task_invocation_info(task_info, task_invocation_info);

        // Add required declarations to the tree
        Nodecl::Utils::prepend_to_enclosing_top_level_location(
            _task_body,
            Nodecl::CxxDef::make(Nodecl::NodeclBase::null(), new_class_symbol));

        Nodecl::Utils::prepend_to_enclosing_top_level_location(
                _task_body,
                Nodecl::List::make(
                    Nodecl::CxxDef::make(Nodecl::NodeclBase::null(), task_invocation_info),
                    Nodecl::ObjectInit::make(task_invocation_info)));

        Nodecl::Utils::append_to_top_level_nodecl(
                 Nodecl::List::make(
                     Nodecl::CxxDef::make(Nodecl::Context::make(nodecl_null(), TL::Scope::get_global_scope()), task_info),
                     Nodecl::ObjectInit::make(task_info)));
    }

    void TaskProperties::create_task_invocation_info(
        TL::Symbol task_info,
        /* out */ TL::Symbol &task_invocation_info)
    {
        TL::Symbol task_invocation_info_struct
            = TL::Scope::get_global_scope().get_symbol_from_name(
                "nanos_task_invocation_info");
        ERROR_CONDITION(
            !task_invocation_info_struct.is_valid(), "Invalid symbol", 0);

        std::string task_invocation_info_name = get_new_name("task_invocation_info");

        task_invocation_info = TL::Scope::get_global_scope().new_symbol(
            task_invocation_info_name);
        task_invocation_info.get_internal_symbol()->kind = SK_VARIABLE;
        symbol_entity_specs_set_is_user_declared(
            task_invocation_info.get_internal_symbol(), 1);
        task_invocation_info.set_type(
            task_invocation_info_struct.get_user_defined_type());
        symbol_entity_specs_set_is_static(
            task_invocation_info.get_internal_symbol(), 1);

        TL::ObjectList<TL::Symbol> task_invocation_fields
            = task_invocation_info_struct.get_type()
                  .get_nonstatic_data_members();
        GetField get_field_task_invocation_info(task_invocation_fields);

        Nodecl::NodeclBase field_invocation_source
            = get_field_task_invocation_info("invocation_source");

        const char *c = locus_to_str(_locus_of_task_creation);
        Nodecl::NodeclBase init_invocation_source = const_value_to_nodecl(
            const_value_make_string_null_ended(c, strlen(c)));

        Nodecl::NodeclBase task_invocation_init = Nodecl::StructuredValue::make(
            Nodecl::List::make(Nodecl::FieldDesignator::make(
                field_invocation_source,
                init_invocation_source,
                field_invocation_source.get_type())),
            Nodecl::StructuredValueBracedImplicit::make(),
            task_info.get_type());
        task_invocation_info.set_value(task_invocation_init);
    }

    namespace {

        template < unsigned int num_arguments>
        TL::Symbol get_fortran_intrinsic_symbol(const std::string &name, const Nodecl::List& actual_arguments, bool is_call)
        {
            // Note that this function is template to avoid to use VLAs in C++ or dynamic memory allocation
            nodecl_t arguments[num_arguments];

            int index = 0;
            for (Nodecl::List::const_iterator it = actual_arguments.begin();
                    it != actual_arguments.end();
                    it++)
            {
                arguments[index++]=it->get_internal_nodecl();
            }
            TL::Symbol intrinsic(
                    fortran_solve_generic_intrinsic_call(
                        fortran_query_intrinsic_name_str(TL::Scope::get_global_scope().get_decl_context(), name.c_str()),
                        arguments,
                        num_arguments,
                        is_call));

            return intrinsic;
        }

        void compute_generic_flag_c(
                Nodecl::NodeclBase opt_expr,
                int default_value,
                int bit,
                // Out
                Nodecl::NodeclBase& flags_expr)
        {
            // If the opt_expr is not present, we create a new expression using the default value
            if (opt_expr.is_null())
                opt_expr = const_value_to_nodecl(const_value_get_signed_int(default_value));

            // Building the expression for the current flag
            Nodecl::NodeclBase current_flag_expr = Nodecl::BitwiseShl::make(
                    Nodecl::Different::make(
                        opt_expr,
                        const_value_to_nodecl(const_value_get_signed_int(0)),
                        TL::Type::get_bool_type()),
                    const_value_to_nodecl(const_value_get_unsigned_int(bit)),
                    get_size_t_type());

            // Finally, we have to combine the expression fo the current flag with the previous ones
            if (!flags_expr.is_null())
            {
                flags_expr = Nodecl::BitwiseOr::make(
                        flags_expr,
                        current_flag_expr,
                        flags_expr.get_type());
            }
            else flags_expr = current_flag_expr;
        }

        Nodecl::NodeclBase compute_generic_flag_fortran(
                TL::Symbol task_flags,
                Nodecl::NodeclBase opt_expr,
                int default_value,
                int bit)
        {
            // If the opt_expr is not present, we create a new expression using the default value
            if (opt_expr.is_null())
                opt_expr = Nodecl::BooleanLiteral::make(
                        TL::Type::get_bool_type(),
                        const_value_get_unsigned_int(default_value));

            Nodecl::List arguments_list = Nodecl::List::make(
                    Nodecl::FortranActualArgument::make(task_flags.make_nodecl()),
                    Nodecl::FortranActualArgument::make(const_value_to_nodecl(const_value_get_signed_int(bit))));

            TL::Symbol intrinsic_ibset = get_fortran_intrinsic_symbol<2>("ibset", arguments_list, /* is_call */ 0);

            Nodecl::FunctionCall ibset_function_call =
                Nodecl::FunctionCall::make(
                        intrinsic_ibset.make_nodecl(),
                        arguments_list,
                        /* alternate_name */ Nodecl::NodeclBase::null(),
                        /* function_form */ Nodecl::NodeclBase::null(),
                        intrinsic_ibset.get_type().returns(),
                        task_flags.get_locus());

            Nodecl::NodeclBase flag_stmt = Nodecl::IfElseStatement::make(
                    /* condition */
                    opt_expr,
                    /* then */
                    Nodecl::List::make(
                        Nodecl::ExpressionStatement::make(
                            Nodecl::Assignment::make(
                                /* lhs */ task_flags.make_nodecl(),
                                /* rhs */ ibset_function_call,
                                /* type */ task_flags.get_type().get_lvalue_reference_to()),
                            task_flags.get_locus())),
                    Nodecl::NodeclBase::null());

            return flag_stmt;
        }

        // This function negates the condition if it's not null
        Nodecl::NodeclBase negate_condition_if_possible(Nodecl::NodeclBase cond)
        {
            if (cond.is_null())
                return cond;

            return Nodecl::LogicalNot::make(cond, TL::Type::get_bool_type());
        }
    }

    void TaskProperties::compute_task_flags(TL::Symbol task_flags, Nodecl::NodeclBase& out_stmts)
    {
        Nodecl::List new_stmts;

        // Note that depending on the base language we compute the flags of a task a bit different:
        //      * C/C++: we compute a new expression that contains all the flags
        //
        //              taskflags = ((final_expr != 0) << 0)  |
        //                          ((!if_expr != 0) << 1)    |
        //                          ((is_taskloop != 0) << 2) |
        //                          ((wait_clause != 0) << 3)
        //
        //      * Fortran: since Fortran doesn't have a simple way to work with
        //        bit fields, we generate several statements:
        //
        //              taskflags = 0;
        //              if (final_expr)  call ibset(taskflags, 0);
        //              if (!if_expr)    call ibset(taskflags, 1);
        //              if (is_taskloop) call ibset(taskflags, 2);
        //              if (wait_clause) call ibset(taskflags, 3);
        //
        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
        {
            Nodecl::NodeclBase task_flags_expr;

            compute_generic_flag_c(_env.final_clause,
                    /* default value */ 0, /* bit */ 0, /* out */ task_flags_expr);

            compute_generic_flag_c(negate_condition_if_possible(_env.if_clause),
                    /* default value */ 0, /* bit */ 1, /* out */ task_flags_expr);

            compute_generic_flag_c(Nodecl::NodeclBase::null(),
                    /* default value */ _env.is_taskloop, /* bit */ 2, /* out */ task_flags_expr);

            compute_generic_flag_c(Nodecl::NodeclBase::null(),
                    /* default value */ _env.wait_clause, /* bit */ 3, /* out */ task_flags_expr);

            new_stmts.append(
                    Nodecl::ExpressionStatement::make(
                        Nodecl::Assignment::make(
                            task_flags.make_nodecl(/* set_ref_type */ 1),
                            task_flags_expr,
                            task_flags.get_type().no_ref().get_lvalue_reference_to()
                            )));
        }
        else // IS_FORTRAN_LANGUAGE
        {
            // Initialize the task_flags variable to zero
            new_stmts.append(
                    Nodecl::ExpressionStatement::make(
                        Nodecl::Assignment::make(
                            task_flags.make_nodecl(/* set_ref_type */ 1),
                            Nodecl::IntegerLiteral::make(
                                TL::Type::get_size_t_type(),
                                const_value_get_signed_int(0)),
                            task_flags.get_type().no_ref().get_lvalue_reference_to())));

            new_stmts.append(
                    compute_generic_flag_fortran(task_flags, _env.final_clause, /* default value */ 0, /* bit */ 0));

            new_stmts.append(
                    compute_generic_flag_fortran(task_flags, negate_condition_if_possible(_env.if_clause), /* default value */ 0, /* bit */ 1));

            new_stmts.append(
                    compute_generic_flag_fortran(task_flags, Nodecl::NodeclBase::null(), /* default value */ _env.is_taskloop, /* bit */ 2));

            new_stmts.append(
                    compute_generic_flag_fortran(task_flags, Nodecl::NodeclBase::null(), /* default value */ _env.wait_clause, /* bit */ 3));
        }
        out_stmts = new_stmts;
    }

    void TaskProperties::create_task_info(
            /* out */
            TL::Symbol &task_info,
            TL::Symbol &task_invocation_info,
            Nodecl::NodeclBase &local_init)
    {
        create_outline_function();
        create_dependences_function();
        create_cost_function();
        create_priority_function();

        TL::Symbol task_info_struct =
            TL::Scope::get_global_scope().get_symbol_from_name("nanos_task_info");

        ERROR_CONDITION(!task_info_struct.is_valid()
                || !(task_info_struct.is_typedef()
                    || task_info_struct.is_class()),
                "Invalid symbol", 0);

        std::string task_info_name = get_new_name("task_info_var");

        if (IS_C_LANGUAGE || IS_FORTRAN_LANGUAGE)
        {
            create_task_info_regular_function(task_info_struct,
                                              task_info_name,
                                              task_info,
                                              task_invocation_info,
                                              local_init);
        }
        else if (IS_CXX_LANGUAGE)
        {
            if (!_related_function.get_type().is_template_specialized_type()
                || (!_related_function.get_type().is_dependent()
                    && (!_related_function.is_member()
                        || !_related_function.get_class_type().is_dependent())))
            {
                create_task_info_nondependent_function(
                        task_info_struct,
                        task_info_name,
                        task_info,
                        task_invocation_info,
                        local_init);
            }
            else
            {
                create_task_info_dependent_function(
                        task_info_struct,
                        task_info_name,
                        task_info,
                        task_invocation_info,
                        local_init);
            }
        }
        else
        {
            internal_error("Code unreachable", 0);
        }

        TL::ObjectList<TL::Symbol> fields = task_info_struct.get_type().get_nonstatic_data_members();
        GetField get_field(fields);

        Nodecl::NodeclBase field_run = get_field("run");

        TL::ObjectList<TL::Type> run_type_params;
        run_type_params.append(TL::Type::get_void_type().get_pointer_to());

        TL::Symbol taskloop_bounds_struct
            = TL::Scope::get_global_scope().get_symbol_from_name("nanos6_taskloop_bounds_t");

        run_type_params.append(taskloop_bounds_struct.get_user_defined_type().get_pointer_to());

        Nodecl::NodeclBase init_run;
        if (_outline_function.is_valid())
        {
            TL::Type run_type =
                TL::Type::get_void_type().get_function_returning(run_type_params).get_pointer_to();

            if (IS_FORTRAN_LANGUAGE)
            {
                init_run = _outline_function_mangled.make_nodecl(/* set_ref_type */ true);
            }
            else
            {
                init_run = _outline_function.make_nodecl(/* set_ref_type */ true);
            }
            init_run = Nodecl::Conversion::make(
                    init_run,
                    run_type);
            init_run.set_text("C");
        }
        else
        {
            init_run = const_value_to_nodecl(const_value_get_signed_int(0));
        }

        Nodecl::NodeclBase field_register_depinfo = get_field("register_depinfo");
        Nodecl::NodeclBase init_register_depinfo;
        TL::Type dep_or_copies_fun_type = TL::Type::get_void_type().get_function_returning(
                TL::ObjectList<TL::Type>(2, TL::Type::get_void_type().get_pointer_to()))
            .get_pointer_to();
        if (_dependences_function.is_valid())
        {
            if (IS_FORTRAN_LANGUAGE)
            {
                init_register_depinfo = _dependences_function_mangled.make_nodecl(/* set_ref_type */ true);
            }
            else
            {
                init_register_depinfo = _dependences_function.make_nodecl(/* set_ref_type */ true);
            }
            init_register_depinfo = Nodecl::Conversion::make(
                    init_register_depinfo,
                    dep_or_copies_fun_type);
            init_register_depinfo.set_text("C");
        }
        else
        {
            init_register_depinfo = const_value_to_nodecl(const_value_get_signed_int(0));
        }

        Nodecl::NodeclBase field_task_label = get_field("task_label");
        Nodecl::NodeclBase init_task_label;
        if (!_env.task_label.empty())
        {
            char* c = xstrdup(_env.task_label.c_str());
            init_task_label = const_value_to_nodecl(
                    const_value_make_string_null_ended(c, strlen(c))
                    );
            DELETE(c);
        }
        else
        {
            init_task_label = const_value_to_nodecl(const_value_get_signed_int(0));
        }

        Nodecl::NodeclBase field_declaration_source = get_field("declaration_source");

        const char* c = locus_to_str(_locus_of_task_declaration);
        Nodecl::NodeclBase init_declaration_source =
            const_value_to_nodecl(const_value_make_string_null_ended(c, strlen(c)));

        Nodecl::NodeclBase field_get_cost = get_field("get_cost");
        Nodecl::NodeclBase init_get_cost;
        if (_cost_function.is_valid())
        {
            TL::Type cost_fun_type =
                TL::Type::get_size_t_type()
                .get_function_returning(TL::ObjectList<TL::Type>(
                            1, TL::Type::get_void_type().get_pointer_to()))
                .get_pointer_to();

            if (IS_FORTRAN_LANGUAGE)
            {
                init_get_cost =
                    _cost_function_mangled.make_nodecl(/* set_ref_type */ true);
            }
            else
            {
                init_get_cost =
                    _cost_function.make_nodecl(/* set_ref_type */ true);
            }

            init_get_cost =
                Nodecl::Conversion::make(init_get_cost, cost_fun_type);
            init_get_cost.set_text("C");
        }
        else
        {
            init_get_cost =
                const_value_to_nodecl(const_value_get_signed_int(0));
        }

        Nodecl::NodeclBase field_get_priority = get_field("get_priority");
        Nodecl::NodeclBase init_get_priority;
        if (_priority_function.is_valid())
        {
            TL::Type priority_fun_type =
                TL::Type::get_size_t_type().get_function_returning(
                        TL::ObjectList<TL::Type>(
                            1, TL::Type::get_void_type().get_pointer_to()))
                .get_pointer_to();

            if (IS_FORTRAN_LANGUAGE)
            {
                init_get_priority =
                    _priority_function_mangled.make_nodecl(/* set_ref_type */ true);
            }
            else
            {
                init_get_priority =
                    _priority_function.make_nodecl(/* set_ref_type */ true);
            }

            init_get_priority = Nodecl::Conversion::make(
                    init_get_priority,
                    priority_fun_type);
            init_get_priority.set_text("C");
        }
        else
        {
            init_get_priority =
                const_value_to_nodecl(const_value_get_signed_int(0));
        }

        TL::ObjectList<Nodecl::NodeclBase> field_init;
        field_init.append(
            Nodecl::FieldDesignator::make(field_run,
                                          init_run,
                                          field_run.get_type()));
        field_init.append(
            Nodecl::FieldDesignator::make(field_register_depinfo,
                                          init_register_depinfo,
                                          field_register_depinfo.get_type()));
        field_init.append(
            Nodecl::FieldDesignator::make(field_get_priority,
                                          init_get_priority,
                                          field_get_priority.get_type()));
        field_init.append(
            Nodecl::FieldDesignator::make(field_task_label,
                                          init_task_label,
                                          field_task_label.get_type()));
        field_init.append(
            Nodecl::FieldDesignator::make(field_declaration_source,
                                          init_declaration_source,
                                          field_declaration_source.get_type()));
        field_init.append(
            Nodecl::FieldDesignator::make(field_get_cost,
                                          init_get_cost,
                                          field_get_cost.get_type()));

        Nodecl::NodeclBase struct_init = Nodecl::StructuredValue::make(
            Nodecl::List::make(field_init),
            Nodecl::StructuredValueBracedImplicit::make(),
            task_info.get_type());

        task_info.set_value(struct_init);

        if (IS_FORTRAN_LANGUAGE)
        {
            // Create a detached symbol with the same name as the real one
            // We need to do that otherwise Fortran codegen attempts to
            // initialize this symbol
            // (We may want to fix this somehow)
            symbol_entity_specs_set_is_static(task_info.get_internal_symbol(),
                                              0);
            //
            scope_entry_t *task_info_shim = NEW0(scope_entry_t);
            task_info_shim->symbol_name
                = task_info.get_internal_symbol()->symbol_name;
            task_info_shim->kind = task_info.get_internal_symbol()->kind;
            task_info_shim->decl_context
                = task_info.get_internal_symbol()->decl_context;
            symbol_entity_specs_set_is_user_declared(task_info_shim, 1);
            // Fake the structure size
            const int size_of_ptr
                = TL::Type::get_void_type().get_pointer_to().get_size();
            ERROR_CONDITION(task_info.get_type().get_size() % size_of_ptr != 0,
                            "Struct size does not divide the size of a pointer",
                            0);
            int num_elements = task_info.get_type().get_size() / size_of_ptr;
            task_info_shim->type_information
                = TL::Type(fortran_choose_int_type_from_kind(size_of_ptr))
                      .get_array_to(
                           const_value_to_nodecl(
                               const_value_get_signed_int(num_elements)),
                           TL::Scope::get_global_scope())
                      .get_internal_type();

            task_info = task_info_shim;

            // Ditto for task_invocation_info
            symbol_entity_specs_set_is_static(
                task_invocation_info.get_internal_symbol(), 0);
            //
            scope_entry_t *task_invocation_info_shim = NEW0(scope_entry_t);
            task_invocation_info_shim->symbol_name
                = task_invocation_info.get_internal_symbol()->symbol_name;
            task_invocation_info_shim->kind
                = task_invocation_info.get_internal_symbol()->kind;
            task_invocation_info_shim->decl_context
                = task_invocation_info.get_internal_symbol()->decl_context;
            symbol_entity_specs_set_is_user_declared(task_invocation_info_shim,
                                                     1);
            ERROR_CONDITION(
                task_invocation_info.get_type().get_size() % size_of_ptr != 0,
                "Struct size does not divide the size of a pointer",
                0);
            num_elements = task_invocation_info.get_type().get_size()
                           / size_of_ptr;
            task_invocation_info_shim->type_information
                = TL::Type(fortran_choose_int_type_from_kind(size_of_ptr))
                      .get_array_to(
                           const_value_to_nodecl(
                               const_value_get_signed_int(num_elements)),
                           TL::Scope::get_global_scope())
                      .get_internal_type();

            task_invocation_info = task_invocation_info_shim;
        }
    }

    TL::Scope TaskProperties::compute_scope_for_environment_structure()
    {
        TL::Scope sc = _related_function.get_scope();
        // We are enclosed by a function because we are an internal subprogram
        if (IS_FORTRAN_LANGUAGE && _related_function.is_nested_function())
        {
            // Get the enclosing function
            TL::Symbol enclosing_function = _related_function.get_scope().get_related_symbol();

            // Update the scope
            sc = enclosing_function.get_scope();
        }

        if (_related_function.is_member())
        {
            // Class scope
            sc = ::class_type_get_inner_context(_related_function.get_class_type().get_internal_type());
        }
        else if (_related_function.is_in_module())
        {
            // Scope of the module
            sc = _related_function.in_module().get_related_scope();
        }

        return sc;
    }

    TL::Symbol TaskProperties::add_field_to_class(TL::Symbol new_class_symbol,
                                                  TL::Scope class_scope,
                                                  const std::string &var_name,
                                                  const locus_t *var_locus,
                                                  bool is_allocatable,
                                                  TL::Type field_type)
    {
        TL::Type new_class_type = new_class_symbol.get_user_defined_type();

        std::string orig_field_name = var_name;

        if (IS_CXX_LANGUAGE && orig_field_name == "this")
        {
            orig_field_name = "_this";
        }

        TL::Symbol field = class_scope.new_symbol(orig_field_name);
        field.get_internal_symbol()->kind = SK_VARIABLE;
        symbol_entity_specs_set_is_user_declared(field.get_internal_symbol(), 1);

        field.set_type( field_type );
        field.get_internal_symbol()->locus = var_locus;

        symbol_entity_specs_set_is_member(field.get_internal_symbol(), 1);
        symbol_entity_specs_set_class_type(field.get_internal_symbol(),
                new_class_type.get_internal_type());
        symbol_entity_specs_set_access(field.get_internal_symbol(), AS_PUBLIC);

        symbol_entity_specs_set_is_allocatable(
                field.get_internal_symbol(), is_allocatable);

        class_type_add_member(
                new_class_type.get_internal_type(),
                field.get_internal_symbol(),
                field.get_internal_symbol()->decl_context,
                /* is_definition */ 1);

        return field;
    }

    namespace
    {
    TL::Type rewrite_type(TL::Type t, TL::Scope scope, Nodecl::Utils::SymbolMap &symbol_map)
    {
        return type_deep_copy(
            t.get_internal_type(),
            scope.get_decl_context(),
            symbol_map.get_symbol_map());
    }

    TL::Type fortran_storage_type_array_descriptor(TL::Type array_type)
    {
        TL::Type void_pointer = TL::Type::get_void_type().get_pointer_to();
        TL::Type suitable_integer
            = fortran_choose_int_type_from_kind(void_pointer.get_size());

        size_t size_of_array_descriptor = fortran_size_of_array_descriptor(
            fortran_get_rank0_type(array_type.get_internal_type()),
            fortran_get_rank_of_type(array_type.get_internal_type()));

        ERROR_CONDITION(
            (size_of_array_descriptor % suitable_integer.get_size()) != 0,
            "The size of the descriptor is not a multiple of the integer type",
            0);

        int num_items = size_of_array_descriptor / suitable_integer.get_size();

        return get_array_type_bounds(
            suitable_integer.get_internal_type(),
            const_value_to_nodecl(const_value_get_signed_int(1)),
            const_value_to_nodecl(const_value_get_signed_int(num_items)),
            TL::Scope::get_global_scope().get_decl_context());
    }

    std::string get_name_for_descriptor(const std::string &var_name)
    {
        Counter &counter
            = CounterManager::get_counter("array-descriptor-copies");
        std::stringstream ss;
        ss << var_name << "_descriptor_" << (int)counter;
        counter++;
        return ss.str();
    }

    // Given an array type, this function returns an array type with descriptor
    //      INTEGER :: V(N, M)  -> INTEGER, WITH_DESC :: V(N, M)
    TL::Type array_type_to_array_with_descriptor_type(TL::Type array_type, TL::Scope sc)
    {
        ERROR_CONDITION(!IS_FORTRAN_LANGUAGE, "This function is only for Fortran", 0);
        ERROR_CONDITION(!array_type.is_array(), "This type should be an array type", 0);

        TL::Type element_type = array_type.array_element();
        if (element_type.is_array())
            element_type = array_type_to_array_with_descriptor_type(element_type, sc);

        Nodecl::NodeclBase lbound, ubound;
        array_type.array_get_bounds(lbound, ubound);
        return element_type.get_array_to_with_descriptor(lbound, ubound, sc);
    }

    }

    void TaskProperties::create_environment_structure(
            /* out */
            TL::Type& data_env_struct,
            Nodecl::NodeclBase& args_size,
            bool &requires_initialization)
    {
        _field_map.clear();

        // By default the arguments structure doesn't require to be initialized
        requires_initialization = false;

        TL::Scope sc = compute_scope_for_environment_structure();

        std::string structure_name;
        if (IS_C_LANGUAGE
                || IS_FORTRAN_LANGUAGE)
        {
            // We need an extra 'struct '
            structure_name = "struct " + get_new_name("nanos_task_args");
        }
        else
        {
            structure_name = get_new_name("nanos_task_args");
        }

        // Create the class symbol
        TL::Symbol new_class_symbol = sc.new_symbol(structure_name);
        symbol_entity_specs_set_is_user_declared(
            new_class_symbol.get_internal_symbol(), 1);

        type_t *new_class_type
            = get_new_class_type(sc.get_decl_context(), TT_STRUCT);

        if (_related_function.get_type().is_template_specialized_type()
            && _related_function.get_type().is_dependent())
        {
            template_parameter_list_t *tpl
                = _related_function.get_type()
                      .template_specialized_type_get_template_parameters()
                      .get_internal_template_parameter_list();
            ERROR_CONDITION(
                tpl == NULL, "There must be template parameters", 0);

            new_class_symbol.get_internal_symbol()->kind = SK_TEMPLATE;
            type_t *template_type = get_new_template_type(
                tpl,
                new_class_type,
                uniquestr(structure_name.c_str()),
                _related_function.get_scope().get_decl_context(),
                _locus_of_task_creation);
            new_class_symbol.set_type(template_type);

            ::template_type_set_related_symbol(
                template_type, new_class_symbol.get_internal_symbol());

            symbol_entity_specs_set_is_user_declared(
                new_class_symbol.get_internal_symbol(), 1);

            new_class_symbol
                = TL::Type(::template_type_get_primary_type(
                               new_class_symbol.get_type().get_internal_type()))
                      .get_symbol();
            new_class_type = new_class_symbol.get_type().get_internal_type();
        }
        else
        {
            new_class_symbol.get_internal_symbol()->kind = SK_CLASS;
        }

        if (sc.is_class_scope())
        {
            symbol_entity_specs_set_is_member(
                new_class_symbol.get_internal_symbol(), 1);
            symbol_entity_specs_set_access(
                new_class_symbol.get_internal_symbol(), AS_PUBLIC);
            symbol_entity_specs_set_class_type(
                new_class_symbol.get_internal_symbol(),
                _related_function.get_class_type().get_internal_type());
            symbol_entity_specs_set_is_defined_inside_class_specifier(
                new_class_symbol.get_internal_symbol(), 1);

            class_type_add_member(
                _related_function.get_class_type().get_internal_type(),
                new_class_symbol.get_internal_symbol(),
                new_class_symbol.get_internal_symbol()->decl_context,
                /* is_definition */ 1);

            class_type_set_enclosing_class_type(new_class_type,
                                                sc.get_related_symbol()
                                                    .get_user_defined_type()
                                                    .get_internal_type());
            set_is_dependent_type(
                new_class_type,
                is_dependent_type(new_class_type)
                    || sc.get_related_symbol().get_type().is_dependent());
        }

        symbol_entity_specs_set_is_user_declared(new_class_symbol.get_internal_symbol(), 1);

        const decl_context_t* class_context = new_class_context(new_class_symbol.get_scope().get_decl_context(),
                new_class_symbol.get_internal_symbol());

        TL::Scope class_scope(class_context);

        class_type_set_inner_context(new_class_type, class_context);

        new_class_symbol.get_internal_symbol()->type_information = new_class_type;

        // It maps each captured symbol with its respective symbol in the arguments structure

        // 1. Create fields for captured symbols
        Nodecl::Utils::SimpleSymbolMap captured_symbols_map;
        for (TL::ObjectList<TL::Symbol>::iterator it = _env.captured_value.begin();
                it != _env.captured_value.end();
                it++)
        {
            TL::Type type_of_field = it->get_type().no_ref();
            bool is_allocatable = it->is_allocatable();

            if (type_of_field.depends_on_nonconstant_values())
            {
                if (IS_CXX_LANGUAGE || IS_C_LANGUAGE)
                    type_of_field = TL::Type::get_void_type().get_pointer_to();
                else
                {
                    // We rewrite the type since it may refer to captured symbols
                    TL::Type updated_type = rewrite_type(type_of_field, class_scope, captured_symbols_map);
                    type_of_field = array_type_to_array_with_descriptor_type(updated_type, sc);
                    is_allocatable = 1;
                }
            }
            else if (type_of_field.is_function())
            {
                if (IS_FORTRAN_LANGUAGE)
                {
                    type_of_field = TL::Type::get_void_type().get_pointer_to();
                }
                else
                {
                    type_of_field = type_of_field.get_pointer_to();
                }
            }
            else
            {
                type_of_field = type_of_field.get_unqualified_type();
            }

            // Fields that require an array descriptor have to be initialized
            if (IS_FORTRAN_LANGUAGE
                    && (
                        (type_of_field.is_array()
                         && type_of_field.array_requires_descriptor())
                        ||
                        (type_of_field.is_pointer()
                            && type_of_field.points_to().is_array()
                            && type_of_field.points_to().array_requires_descriptor())))
            {
                requires_initialization = true;
            }

            TL::Symbol field = add_field_to_class(
                    new_class_symbol,
                    class_scope,
                    it->get_name(),
                    it->get_locus(),
                    is_allocatable,
                    type_of_field);

            _field_map[*it] = field;
            captured_symbols_map.add_map(*it, field);
        }

        // 2. Create fields for shared symbols
        for (TL::ObjectList<TL::Symbol>::iterator it = _env.shared.begin();
                it != _env.shared.end();
                it++)
        {
            TL::Type type_of_field = it->get_type().no_ref();
            if (IS_FORTRAN_LANGUAGE)
            {
                type_of_field = TL::Type::get_void_type().get_pointer_to();
                if (it->get_type().no_ref().is_array()
                    && it->get_type().no_ref().array_requires_descriptor())
                {
                    TL::Symbol field = add_field_to_class(
                        new_class_symbol,
                        class_scope,
                        get_name_for_descriptor(it->get_name()),
                        it->get_locus(),
                        /* is_allocatable */ false,
                        fortran_storage_type_array_descriptor(
                            it->get_type().no_ref()));

                    _array_descriptor_map[*it] = field;
                }
            }
            else
            {
                if (type_of_field.depends_on_nonconstant_values())
                {
                    type_of_field = TL::Type::get_void_type().get_pointer_to();
                }
                else
                {
                    type_of_field = type_of_field.get_pointer_to();
                }
            }

            TL::Symbol field = add_field_to_class(
                    new_class_symbol,
                    class_scope,
                    it->get_name(),
                    it->get_locus(),
                    /* is_allocatable */ false,
                    type_of_field);

            _field_map[*it] = field;
        }

        // 3. Create fields for reduction symbols
        for (TL::ObjectList<ReductionItem>::const_iterator it = _env.reduction.begin();
                it != _env.reduction.end();
                it++)
        {
            const ReductionItem& curr_red_item = *it;

            // First we add the pointer to the original variable
            {
                TL::Type type_of_field = curr_red_item.reduction_type.no_ref();
                if (IS_FORTRAN_LANGUAGE)
                    type_of_field = TL::Type::get_void_type().get_pointer_to();
                else
                    type_of_field = type_of_field.get_pointer_to();

                TL::Symbol field = add_field_to_class(
                        new_class_symbol,
                        class_scope,
                        curr_red_item.symbol.get_name(),
                        curr_red_item.symbol.get_locus(),
                        /* is_allocatable */ false,
                        type_of_field);

                _field_map[curr_red_item.symbol] = field;
            }

            // Second, we add the local variable
            {
                TL::Type type_of_field = curr_red_item.reduction_type;
                TL::Symbol field = add_field_to_class(
                        new_class_symbol,
                        class_scope,
                        curr_red_item.symbol.get_name() + "_local_red",
                        curr_red_item.symbol.get_locus(),
                        /* is_allocatable */ false,
                        type_of_field);

                // Note that we do not explicitly map this field to the original list item!
            }
        }

        // 4. Create fields for private allocatables symbols
        if (IS_FORTRAN_LANGUAGE)
        {
            for (TL::ObjectList<TL::Symbol>::iterator it = _env.private_.begin();
                    it != _env.private_.end();
                    it++)
            {
                if (!symbol_entity_specs_get_is_allocatable(it->get_internal_symbol()))
                    continue;


                // When we are privatizing an allocatable array we add an
                // allocatable component to the arguments structure. For this
                // reason we must initialize the storage for the arguments!
                requires_initialization = true;

                TL::Symbol field = add_field_to_class(
                    new_class_symbol,
                    class_scope,
                    it->get_name(),
                    it->get_locus(),
                    /* is_allocatble */ 1,
                    it->get_type().no_ref().get_unqualified_type());

                _field_map[*it] = field;
            }
        }


        nodecl_t nodecl_output = nodecl_null();
        finish_class_type(new_class_type,
                ::get_user_defined_type(new_class_symbol.get_internal_symbol()),
                sc.get_decl_context(),
                _locus_of_task_creation,
                &nodecl_output);
        set_is_complete_type(new_class_type, /* is_complete */ 1);
        set_is_complete_type(get_actual_class_type(new_class_type), /* is_complete */ 1);

        _info_structure
            = data_env_struct
            = new_class_symbol.get_user_defined_type();


        // Computing the size of the arguments structure
        {

            // This nodecl represents the size of the structure of arguments
            Nodecl::NodeclBase basic_size;
            if (new_class_symbol.get_type().is_dependent())
            {
                basic_size = Nodecl::Sizeof::make(
                        Nodecl::Type::make(_info_structure, _locus_of_task_creation),
                        Nodecl::NodeclBase::null(),
                        TL::Type::get_size_t_type(),
                        _locus_of_task_creation);
            }
            else
            {
                basic_size = const_value_to_nodecl_with_basic_type(
                        const_value_get_integer(
                            _info_structure.get_size(),
                            /* bytes */ type_get_size(get_size_t_type()),
                            /* sign */ 0),
                        get_size_t_type());
            }

            // This nodecl represents the extra storage that the runtime has to
            // allocate contiguosly to the arguments structure to support VLAs
            Nodecl::NodeclBase extra_storage = const_value_to_nodecl(const_value_get_signed_int(0));
            for (TL::ObjectList<TL::Symbol>::iterator it = _env.captured_value.begin();
                    it != _env.captured_value.end();
                    it++)
            {
                if (it->get_type().depends_on_nonconstant_values())
                {
                    if (IS_CXX_LANGUAGE || IS_C_LANGUAGE)
                    {
                        Nodecl::NodeclBase size_of_array = Nodecl::Add::make(
                                const_value_to_nodecl(const_value_get_signed_int(VLA_OVERALLOCATION_ALIGN)),
                                Nodecl::Sizeof::make(
                                    Nodecl::Type::make(it->get_type()),
                                    Nodecl::NodeclBase::null(),
                                    get_size_t_type()),
                                get_size_t_type());

                        extra_storage = Nodecl::Add::make(
                                extra_storage,
                                size_of_array,
                                size_of_array.get_type());
                    }
                }
            }

            // Finally, we compute the real size of our arguments
            args_size = Nodecl::Add::make(basic_size, extra_storage, basic_size.get_type());
        }

        if (IS_CXX_LANGUAGE)
        {
            Nodecl::Utils::prepend_to_enclosing_top_level_location(
                    _task_body,
                    Nodecl::CxxDef::make(
                        Nodecl::NodeclBase::null(),
                        new_class_symbol));
        }
    }

    namespace {
        struct GenerateParamsNames
        {
            int x;
            GenerateParamsNames() : x(0) { }
            std::string operator()()
            {
                std::stringstream ss;
                ss << "p_" << x;
                x++;
                return ss.str();
            }
        };

        struct SolveParamNames
        {
            TL::Scope sc;
            SolveParamNames(TL::Scope sc_)
                : sc(sc_) { }
            Nodecl::NodeclBase operator()(const std::string& str)
            {
                TL::Symbol sym = sc.get_symbol_from_name(str);
                ERROR_CONDITION(!sym.is_valid(), "Symbol '%s' not found", sym.get_name().c_str());

                return sym.make_nodecl(/* set_ref_type */ true);
            }
        };

        struct AddParameter
        {
            private:
                TL::ObjectList<std::string> &_parameter_names;
                TL::ObjectList<TL::Type> &_parameter_types;
                std::map<TL::Symbol, std::string> &_symbols_to_param_names;
                bool _map_reduction_local_symbol;

            public:
                AddParameter(
                        TL::ObjectList<std::string> &parameter_names,
                        TL::ObjectList<TL::Type> &parameter_types,
                        std::map<TL::Symbol, std::string> &symbols_to_param_names,
                        bool map_reduction_local_symbol = true)
                    : _parameter_names(parameter_names), _parameter_types(parameter_types),
                    _symbols_to_param_names(symbols_to_param_names),
                    _map_reduction_local_symbol(map_reduction_local_symbol)
                {}

                void handle_symbol(TL::Symbol sym, const std::string& name, bool add_to_map)
                {
                    std::string fixed_name = name;
                    if (IS_CXX_LANGUAGE && name == "this")
                        fixed_name = "_this";

                    if (add_to_map)
                        _symbols_to_param_names[sym] = fixed_name;

                    _parameter_names.append(fixed_name);
                    _parameter_types.append(sym.get_type().no_ref().get_lvalue_reference_to());
                }

                void operator()(TL::Symbol sym)
                {
                    handle_symbol(sym, sym.get_name(), /* add_to_map */ true);
                }

                void operator()(const ReductionItem& red_item)
                {
                    TL::Symbol sym = red_item.symbol;
                    handle_symbol(sym, sym.get_name(), !_map_reduction_local_symbol);
                    handle_symbol(sym, sym.get_name() + "_local_red", _map_reduction_local_symbol);
                }
        };

        bool type_is_runtime_sized(TL::Type t)
        {
            if (!t.is_valid())
                return false;

            if (t.is_any_reference())
                return type_is_runtime_sized(t.no_ref());
            else if (t.is_pointer())
                return type_is_runtime_sized(t.points_to());
            else if (t.is_array())
            {
                if (type_is_runtime_sized(t.array_element()))
                    return true;

                if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
                {
                    Nodecl::NodeclBase size = t.array_get_size();
                    return !size.is_null() && !size.is_constant();
                }
                else if (IS_FORTRAN_LANGUAGE)
                {
                    Nodecl::NodeclBase lbound, ubound;
                    t.array_get_bounds(lbound, ubound);

                    return (!lbound.is_null() && !lbound.is_constant())
                        || (!ubound.is_null() && !ubound.is_constant());
                }
                else
                {
                    internal_error("Code unreachable", 0);
                }
            }
            return false;
        }

        struct MapSymbols
        {
            private:
                const TL::Scope &_function_scope;
                const std::map<TL::Symbol, std::string> &_symbols_to_param_names;

                TL::ObjectList<TL::Symbol> &_parameters_to_update_type;
                Nodecl::Utils::SimpleSymbolMap &_symbol_map;

            public:
                MapSymbols(
                        const TL::Scope &function_scope,
                        const std::map<TL::Symbol, std::string> &symbols_to_param_names,
                        /* out */ Nodecl::Utils::SimpleSymbolMap &symbol_map,
                        /* out */ TL::ObjectList<TL::Symbol> &parameter_to_update_type)
                    : _function_scope(function_scope),
                    _symbols_to_param_names(symbols_to_param_names),
                    _parameters_to_update_type(parameter_to_update_type),
                    _symbol_map(symbol_map)
                {}

                void compute_mapping(TL::Symbol sym)
                {
                    std::map<TL::Symbol, std::string>::const_iterator it_param_name = _symbols_to_param_names.find(sym);
                    ERROR_CONDITION(it_param_name == _symbols_to_param_names.end(),
                            "Symbol '%s' not mapped",sym.get_name().c_str());

                    TL::Symbol param_sym = _function_scope.get_symbol_from_name(it_param_name->second);


                    ERROR_CONDITION(!param_sym.is_valid()
                            || (!param_sym.is_parameter() && !param_sym.is_saved_expression()),
                            "Invalid symbol for name '%s'", it_param_name->second.c_str());

                    _symbol_map.add_map(sym, param_sym);

                    // Propagate TARGET attribute
                    if (sym.is_target())
                        symbol_entity_specs_set_is_target(param_sym.get_internal_symbol(), 1);

                    // Propagate ALLOCATABLE attribute
                    if (sym.is_allocatable())
                        symbol_entity_specs_set_is_allocatable(param_sym.get_internal_symbol(), 1);

                    if (type_is_runtime_sized(param_sym.get_type()))
                        _parameters_to_update_type.append(param_sym);
                }

                void operator()(TL::Symbol sym)
                {
                    compute_mapping(sym);
                }

                void operator()(const ReductionItem& red_item)
                {
                    compute_mapping(red_item.symbol);
                }
        };

        // It updates the the type of the 'function' symbol if the type of any of its parameters is runtime sized
        void update_function_type_if_needed(
                TL::Symbol function,
                TL::ObjectList<TL::Symbol>& parameters_to_update_type,
                Nodecl::Utils::SimpleSymbolMap& symbol_map)
        {
            if (parameters_to_update_type.empty())
                return;

            TL::Scope function_inside_scope = function.get_related_scope();

            // Now fix the types of runtime sized types prior anything else
            for (TL::ObjectList<TL::Symbol>::iterator it = parameters_to_update_type.begin();
                    it != parameters_to_update_type.end();
                    it++)
            {
                it->set_type(rewrite_type(it->get_type(), function_inside_scope, symbol_map));
            }

            TL::ObjectList<TL::Type> updated_param_types
                = function.get_related_symbols().map<TL::Type>(&TL::Symbol::get_type);

            function.set_type(
                    TL::Type::get_void_type().get_function_returning(updated_param_types));
        }

        // This function computes the loop control that we should emit for a taskloop construct. It should be
        //
        //      for (induction_variable = taskloop_bounds.lower_bound;
        //              induction_variable < taskloop_bounds.upper_bound;
        //              induction_variable++)
        //      {}
        Nodecl::NodeclBase compute_taskloop_loop_control(
                const TL::Symbol& taskloop_bounds,
                const TL::Symbol& induction_variable)
        {
            Nodecl::NodeclBase loop_control;

            TL::ObjectList<TL::Symbol> nonstatic_data_members = taskloop_bounds.get_type().no_ref().get_nonstatic_data_members();
            GetField get_field(nonstatic_data_members);

            Nodecl::NodeclBase field = get_field("lower_bound");
            Nodecl::NodeclBase taskloop_lower_bound =
                Nodecl::ClassMemberAccess::make(
                        taskloop_bounds.make_nodecl(/*set_ref_type*/ true),
                        field,
                        /*member literal*/ Nodecl::NodeclBase::null(),
                        field.get_type());

            field = get_field("upper_bound");
            Nodecl::NodeclBase taskloop_upper_bound =
                Nodecl::ClassMemberAccess::make(
                        taskloop_bounds.make_nodecl(/*set_ref_type*/ true),
                        field,
                        /*member literal*/ Nodecl::NodeclBase::null(),
                        field.get_type());

            if (IS_C_LANGUAGE
                    || IS_CXX_LANGUAGE)
            {
                Nodecl::NodeclBase init =
                    Nodecl::Assignment::make(
                            induction_variable.make_nodecl(/* set_ref_type */ true),
                            taskloop_lower_bound,
                            taskloop_lower_bound.get_type());

                Nodecl::NodeclBase cond =
                    Nodecl::LowerThan::make(
                            induction_variable.make_nodecl(/* set_ref_type */ true),
                            taskloop_upper_bound,
                            taskloop_upper_bound.get_type());

                Nodecl::NodeclBase step =
                    Nodecl::Preincrement::make(
                            induction_variable.make_nodecl(/*set_ref_type*/ true),
                            induction_variable.get_type());

                loop_control = Nodecl::LoopControl::make(Nodecl::List::make(init), cond, step);
            }
            else // IS_FORTRAN_LANGUAGE
            {
                loop_control = Nodecl::RangeLoopControl::make(
                        induction_variable.make_nodecl(/*set_ref_type*/ true),
                        taskloop_lower_bound,
                        Nodecl::Minus::make(
                            taskloop_upper_bound,
                            const_value_to_nodecl(const_value_get_signed_int(1)),
                            taskloop_upper_bound.get_type().no_ref()),
                        /* step */ Nodecl::NodeclBase::null());
            }
            return loop_control;
        }

        TL::Symbol compute_mangled_function_symbol_from_symbol(const TL::Symbol &sym)
        {
            ERROR_CONDITION(sym.get_internal_symbol()->kind != SK_FUNCTION,
                    "Unexpected symbol kind", 0);

            // For Fortran we will generate a global variable in C, but this
            // requires us to use the proper mangling.
            const char* mangled_name =
                ::fortran_mangle_symbol(sym.get_internal_symbol());

            // Create a detached symbol that looks like it comes from the global scope
            TL::Symbol mangled_symbol = NEW0(scope_entry_t);
            mangled_symbol.get_internal_symbol()->symbol_name = mangled_name;
            mangled_symbol.get_internal_symbol()->decl_context
                = TL::Scope::get_global_scope().get_decl_context();
            mangled_symbol.get_internal_symbol()->kind =
                sym.get_internal_symbol()->kind;

            // Fake symbol type as "void fun(void)" to avoid matching problems with argument types,
            // we will need to cast it back to original type when used
            mangled_symbol.get_internal_symbol()->type_information =
                TL::Type::get_void_type().get_function_returning(TL::ObjectList<TL::Type>())
                .get_internal_type();

            symbol_entity_specs_set_is_user_declared(
                    mangled_symbol.get_internal_symbol(), 1);

            return mangled_symbol;
        }
    }


    void TaskProperties::create_outline_function()
    {
        // Skip this function if the current task comes from a taskwait depend
        if (_env.is_taskwait_dep)
            return;

        // Unpacked function
        TL::ObjectList<std::string> unpack_parameter_names;
        TL::ObjectList<TL::Type> unpack_parameter_types;
        std::map<TL::Symbol, std::string> symbols_to_param_names;

        std::string unpacked_name = get_new_name("nanos6_unpack");

        AddParameter add_params_functor(
                /* out */ unpack_parameter_names,
                /* out */ unpack_parameter_types,
                /* out */ symbols_to_param_names);

        _env.captured_value.map(add_params_functor);
        _env.shared.map(add_params_functor);
        _env.reduction.map(add_params_functor);
        _env.private_.filter(&TL::Symbol::is_allocatable).map(add_params_functor);

        if (_env.is_taskloop)
        {
            TL::Symbol taskloop_bounds_struct
                = TL::Scope::get_global_scope().get_symbol_from_name("nanos6_taskloop_bounds_t");

            unpack_parameter_names.append("taskloop_bounds");
            unpack_parameter_types.append(taskloop_bounds_struct.get_user_defined_type().get_lvalue_reference_to());
        }

        TL::Symbol unpacked_function
            = SymbolUtils::new_function_symbol(
                    _related_function,
                    unpacked_name,
                    TL::Type::get_void_type(),
                    unpack_parameter_names,
                    unpack_parameter_types);

        Nodecl::NodeclBase unpacked_function_code, unpacked_empty_stmt;
        SymbolUtils::build_empty_body_for_function(
                unpacked_function,
                unpacked_function_code,
                unpacked_empty_stmt);
        Nodecl::Utils::append_to_top_level_nodecl(unpacked_function_code);

        TL::Scope unpacked_inside_scope = unpacked_function.get_related_scope();
        // Prepare deep copy and remember those parameters that need fixup
        Nodecl::Utils::SimpleSymbolMap symbol_map;
        TL::ObjectList<TL::Symbol> parameters_to_update_type;

        MapSymbols map_symbols_functor(
                unpacked_inside_scope,
                symbols_to_param_names,
                /* out */ symbol_map,
                /* out */ parameters_to_update_type);

        _env.captured_value.map(map_symbols_functor);
        _env.shared.map(map_symbols_functor);
        _env.reduction.map(map_symbols_functor);
        _env.private_.filter(&TL::Symbol::is_allocatable).map(map_symbols_functor);

        update_function_type_if_needed(
                unpacked_function, parameters_to_update_type, symbol_map);

        Nodecl::List extra_decls;
        for (TL::ObjectList<TL::Symbol>::iterator it = _env.private_.begin();
                it != _env.private_.end();
                it++)
        {
            // Allocatable symbols are treated specially
            if (symbol_entity_specs_get_is_allocatable(it->get_internal_symbol()))
                continue;

            TL::Symbol priv = unpacked_inside_scope.new_symbol(it->get_name());
            priv.get_internal_symbol()->kind = SK_VARIABLE;
            symbol_entity_specs_set_is_user_declared(priv.get_internal_symbol(), 1);
            priv.set_type(rewrite_type(it->get_type().no_ref(), unpacked_inside_scope, symbol_map));

            symbol_map.add_map(*it, priv);

            if (IS_CXX_LANGUAGE)
            {
                extra_decls.append(
                        Nodecl::CxxDef::make(
                            Nodecl::NodeclBase::null(),
                            priv,
                            it->get_locus()));
            }
        }

        if (!extra_decls.is_null())
            unpacked_empty_stmt.prepend_sibling(extra_decls);

        Nodecl::List nested_functions;
        if (IS_FORTRAN_LANGUAGE)
        {
            Nodecl::Utils::Fortran::ExtraDeclsVisitor fun_visitor(
                    symbol_map,
                    unpacked_inside_scope,
                    _related_function);
            fun_visitor.insert_extra_symbols(_task_body);

            if (_related_function.is_in_module())
            {
                TL::Symbol in_module = _related_function.in_module();
                Nodecl::Utils::Fortran::append_used_modules(
                        _task_body.retrieve_context(),
                        in_module.get_related_scope());
            }

            Nodecl::Utils::Fortran::append_used_modules(
                    _task_body.retrieve_context(),
                    unpacked_inside_scope);

            if (_related_function.is_nested_function())
            {
                TL::Symbol enclosing_function = _related_function.get_scope().get_related_symbol();

                ERROR_CONDITION(!enclosing_function.is_valid()
                        || !(enclosing_function.is_function()
                            || enclosing_function.is_fortran_main_program()),
                        "Invalid enclosing symbol of nested function", 0);

                Nodecl::Utils::Fortran::append_used_modules(
                        enclosing_function.get_related_scope(),
                        unpacked_inside_scope);
            }

            fortran_add_types(unpacked_inside_scope);

            // Now get all the needed internal functions and duplicate them in the outline
            Nodecl::Utils::Fortran::InternalFunctions internal_functions;
            internal_functions.walk(_task_body);

            nested_functions = duplicate_internal_subprograms(internal_functions.function_codes,
                    unpacked_inside_scope,
                    symbol_map);
        }
        unpacked_empty_stmt.append_sibling(nested_functions);

        handle_task_reductions(unpacked_inside_scope, unpacked_empty_stmt);

        if (_env.is_taskloop)
        {
            ERROR_CONDITION(!_task_body.is<Nodecl::List>(), "Unexpected node\n", 0);
            Nodecl::NodeclBase stmt = _task_body.as<Nodecl::List>().front();
            ERROR_CONDITION(!stmt.is<Nodecl::Context>(), "Unexpected node\n", 0);
            stmt = stmt.as<Nodecl::Context>().get_in_context().as<Nodecl::List>().front();
            ERROR_CONDITION(!stmt.is<Nodecl::ForStatement>(), "Unexpected node\n", 0);

            TL::ForStatement for_stmt(stmt.as<Nodecl::ForStatement>());

            TL::Symbol ind_var = for_stmt.get_induction_variable();
            TL::Symbol taskloop_bounds = unpacked_inside_scope.get_symbol_from_name("taskloop_bounds");

            for_stmt.set_loop_header(compute_taskloop_loop_control(taskloop_bounds, ind_var));
        }

        // Deep copy the body
        Nodecl::NodeclBase body = Nodecl::Utils::deep_copy(_task_body, unpacked_inside_scope, symbol_map);
        unpacked_empty_stmt.replace(body);

        if (IS_CXX_LANGUAGE
                && !_related_function.is_member())
        {
            Nodecl::Utils::prepend_to_enclosing_top_level_location(
                    _task_body,
                    Nodecl::CxxDecl::make(
                        Nodecl::Context::make(
                            Nodecl::NodeclBase::null(),
                            _related_function.get_scope()),
                        unpacked_function));
        }


        // Outline function
        std::string ol_name = get_new_name("nanos6_ol");

        TL::ObjectList<std::string> ol_param_names;
        TL::ObjectList<TL::Type>    ol_param_types;

        ol_param_names.append("arg");
        ol_param_types.append(_info_structure.get_lvalue_reference_to());

        TL::Symbol taskloop_bounds_struct
            = TL::Scope::get_global_scope().get_symbol_from_name("nanos6_taskloop_bounds_t");

        ol_param_names.append("taskloop_bounds");
        ol_param_types.append(taskloop_bounds_struct.get_user_defined_type().get_lvalue_reference_to());

        _outline_function
            = SymbolUtils::new_function_symbol(
                    _related_function,
                    ol_name,
                    TL::Type::get_void_type(),
                    ol_param_names,
                    ol_param_types);

        if (IS_FORTRAN_LANGUAGE)
        {
            _outline_function_mangled =
                compute_mangled_function_symbol_from_symbol(_outline_function);
        }

        Nodecl::NodeclBase outline_function_code, outline_empty_stmt;
        SymbolUtils::build_empty_body_for_function(
                _outline_function,
                outline_function_code,
                outline_empty_stmt);

        TL::Scope outline_inside_scope = outline_empty_stmt.retrieve_context();
        TL::Symbol arg = outline_inside_scope.get_symbol_from_name("arg");
        ERROR_CONDITION(!arg.is_valid(), "Invalid symbol", 0);
        ERROR_CONDITION(!arg.is_parameter(), "Invalid symbol", 0);

        // Prepare the call to the unpacked function
        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
        {
            Nodecl::List args;

            // 1. Visiting captured symbols
            for (TL::ObjectList<TL::Symbol>::iterator it = _env.captured_value.begin();
                    it != _env.captured_value.end();
                    it++)
            {
                ERROR_CONDITION(_field_map.find(*it) == _field_map.end(), "Symbol is not mapped", 0);

                Nodecl::NodeclBase argument = Nodecl::ClassMemberAccess::make(
                        arg.make_nodecl(/* set_ref_type */ true),
                        _field_map[*it].make_nodecl(),
                        /* member_literal */ Nodecl::NodeclBase::null(),
                        _field_map[*it].get_type().no_ref().get_lvalue_reference_to());

                if (it->get_type().depends_on_nonconstant_values() &&
                        (it->get_type().no_ref().is_array() ||
                         it->get_type().no_ref().is_pointer()))
                {
                    // A conversion between pointer type (from argument) and
                    // array type (from parameter) is required. This is done by
                    // getting a reference (&) from the argument, casting it to a
                    // pointer to the array type, and dereferencing (*) it afterwards.

                    TL::Type param_type = rewrite_type_using_args(
                            arg, it->get_type().no_ref(), TL::ObjectList<TL::Symbol>());

                    Nodecl::NodeclBase cast;
                    argument = Nodecl::Dereference::make(
                            cast = Nodecl::Conversion::make(
                                Nodecl::Reference::make(
                                    argument,
                                    _field_map[*it].get_type().get_pointer_to()),
                                param_type.get_pointer_to()),
                            param_type.get_lvalue_reference_to());

                    cast.set_text("C");
                }
                args.append(argument);

                if (it->get_type().is_dependent()
                        || (it->get_type().no_ref().is_class() && !it->get_type().no_ref().is_pod()))
                {
                    argument.set_is_type_dependent(1);
                    TL::Source src;

                    src << "{"
                        <<      "typedef " << as_type(it->get_type().no_ref().get_unqualified_type()) << " DepType;"
                        <<      "( " << as_expression(argument.shallow_copy()) << ").~DepType();"
                        << "}"
                        ;

                    outline_empty_stmt.append_sibling(src.parse_statement(outline_inside_scope));
                }
            }

            // 2. Visiting shared symbols
            for (TL::ObjectList<TL::Symbol>::iterator it = _env.shared.begin();
                    it != _env.shared.end();
                    it++)
            {
                ERROR_CONDITION(_field_map.find(*it) == _field_map.end(), "Symbol is not mapped", 0);

                Nodecl::NodeclBase argument = Nodecl::ClassMemberAccess::make(
                        arg.make_nodecl(/* set_ref_type */ true),
                        _field_map[*it].make_nodecl(),
                        /* member_literal */ Nodecl::NodeclBase::null(),
                        _field_map[*it].get_type().get_lvalue_reference_to());

                if (it->get_type().depends_on_nonconstant_values())
                {
                    TL::Type cast_type_type = rewrite_type_using_args(arg,
                            it->get_type().no_ref().get_pointer_to(),
                            TL::ObjectList<TL::Symbol>());

                    argument = Nodecl::Conversion::make(argument, cast_type_type);
                    argument.set_text("C");
                }

                args.append(Nodecl::Dereference::make(
                            argument,
                            argument.get_type().no_ref().points_to().get_lvalue_reference_to()));
            }

            // 3. Visiting reduction symbols
            for (TL::ObjectList<ReductionItem>::const_iterator it = _env.reduction.begin();
                    it != _env.reduction.end();
                    it++)
            {
                ERROR_CONDITION(_field_map.find(it->symbol) == _field_map.end(), "Symbol is not mapped", 0);

                // 1st. Adding the original reduction variable as an argument
                {
                    args.append(
                            Nodecl::Dereference::make(
                                Nodecl::ClassMemberAccess::make(
                                    arg.make_nodecl(/* set_ref_type */ true),
                                    _field_map[it->symbol].make_nodecl(),
                                    /* member_literal */ Nodecl::NodeclBase::null(),
                                    _field_map[it->symbol].get_type().get_lvalue_reference_to()),
                                _field_map[it->symbol].get_type().points_to().get_lvalue_reference_to())
                            );
                }

                // 2nd. Adding the task local variable as an argument
                {
                    TL::Scope inner_class_context(
                            class_type_get_inner_context(_info_structure.get_internal_type()));

                    TL::Symbol local_symbol = inner_class_context.get_symbol_from_name(it->symbol.get_name() + "_local_red");
                    TL::Type expr_type = local_symbol.get_type().no_ref().get_lvalue_reference_to();
                    args.append(
                            Nodecl::ClassMemberAccess::make(
                                arg.make_nodecl(/* set_ref_type */ true),
                                local_symbol.make_nodecl(),
                                /* member_literal */ Nodecl::NodeclBase::null(),
                                expr_type));
                }
            }

            if (_env.is_taskloop)
            {
                TL::Symbol bounds = outline_inside_scope.get_symbol_from_name("taskloop_bounds");
                args.append(bounds.make_nodecl(/* set_ref_type */ true));
            }

            // Make sure we explicitly pass template arguments to the
            // unpacked function
            Nodecl::NodeclBase function_form;
            if (unpacked_function.get_type().is_template_specialized_type())
            {
                function_form = Nodecl::CxxFunctionFormTemplateId::make();
                function_form.set_template_parameters(
                    unpacked_function.get_type()
                        .template_specialized_type_get_template_arguments());
            }

            Nodecl::NodeclBase call_to_unpacked
                = Nodecl::ExpressionStatement::make(Nodecl::FunctionCall::make(
                    unpacked_function.make_nodecl(/* set_ref_type */ true),
                    args,
                    /* alternate_name */ Nodecl::NodeclBase::null(),
                    function_form,
                    get_void_type()));

            outline_empty_stmt.prepend_sibling(call_to_unpacked);
        }
        else if (IS_FORTRAN_LANGUAGE)
        {
            fortran_add_types(outline_inside_scope);

            std::string forwarded_name = get_new_name("nanos6_fwd");

            TL::ObjectList<std::string> forwarded_parameter_names;
            TL::ObjectList<TL::Type> forwarded_parameter_types;

            forwarded_parameter_names.append("ol");
            forwarded_parameter_types.append(TL::Type::get_void_type().get_pointer_to());

            Nodecl::List args;

            args.append(
                    Nodecl::Reference::make(
                        unpacked_function.make_nodecl(/* set_ref_type */ true),
                        unpacked_function.get_type().get_pointer_to()));

            Nodecl::List deallocate_exprs;
            std::map<std::string, TL::Symbol> names_to_fields;

            // 1. Visiting captured symbols
            for (TL::ObjectList<TL::Symbol>::iterator it = _env.captured_value.begin();
                    it != _env.captured_value.end();
                    it++)
            {
                ERROR_CONDITION(_field_map.find(*it) == _field_map.end(), "Symbol is not mapped", 0);
                TL::Symbol field = _field_map[*it];

                names_to_fields[field.get_name()] = field;

                forwarded_parameter_names.append(field.get_name());
                forwarded_parameter_types.append(field.get_type().get_lvalue_reference_to());

                Nodecl::NodeclBase class_member_access =  Nodecl::ClassMemberAccess::make(
                        arg.make_nodecl(/* set_ref_type */ true),
                        _field_map[*it].make_nodecl(),
                        /* member_literal */ Nodecl::NodeclBase::null(),
                        _field_map[*it].get_type().get_lvalue_reference_to());
                args.append(class_member_access);

                // Finally, if the field symbol is an allocatable, we have to deallocate it.
                // Note that this copy was created when we captured its value.
                if (field.is_allocatable())
                {
                    deallocate_exprs.append(class_member_access.shallow_copy());
                }

            }

            // 2. Visiting shared symbols
            for (TL::ObjectList<TL::Symbol>::iterator it = _env.shared.begin();
                    it != _env.shared.end();
                    it++)
            {
                ERROR_CONDITION(_field_map.find(*it) == _field_map.end(), "Symbol is not mapped", 0);
                TL::Symbol field = _field_map[*it];

                names_to_fields[field.get_name()] = field;

                forwarded_parameter_names.append(field.get_name());
                forwarded_parameter_types.append(field.get_type());

                args.append(
                        Nodecl::ClassMemberAccess::make(
                            arg.make_nodecl(/* set_ref_type */ true),
                            _field_map[*it].make_nodecl(),
                            /* member_literal */ Nodecl::NodeclBase::null(),
                            _field_map[*it].get_type().get_lvalue_reference_to()));
            }

            // 3. Visiting reduction symbols
            for (TL::ObjectList<ReductionItem>::const_iterator it = _env.reduction.begin();
                    it != _env.reduction.end();
                    it++)
            {
                ERROR_CONDITION(_field_map.find(it->symbol) == _field_map.end(), "Symbol is not mapped", 0);

                {
                    TL::Symbol field = _field_map[it->symbol];

                    names_to_fields[field.get_name()] = field;

                    forwarded_parameter_names.append(field.get_name());
                    forwarded_parameter_types.append(field.get_type());

                    args.append(
                            Nodecl::ClassMemberAccess::make(
                        arg.make_nodecl(/* set_ref_type */ true),
                                _field_map[it->symbol].make_nodecl(),
                                /* member_literal */ Nodecl::NodeclBase::null(),
                                _field_map[it->symbol].get_type().get_lvalue_reference_to()));
                }

                {
                    TL::Scope inner_class_context(
                            class_type_get_inner_context(_info_structure.get_internal_type()));

                    TL::Symbol local_symbol = inner_class_context.get_symbol_from_name(it->symbol.get_name() + "_local_red");
                    TL::Type expr_type = local_symbol.get_type().no_ref().get_lvalue_reference_to();

                    names_to_fields[local_symbol.get_name()] = local_symbol;

                    forwarded_parameter_names.append(local_symbol.get_name());
                    forwarded_parameter_types.append(expr_type);

                    args.append(
                            Nodecl::ClassMemberAccess::make(
                                arg.make_nodecl(/* set_ref_type */ true),
                                local_symbol.make_nodecl(),
                                /* member_literal */ Nodecl::NodeclBase::null(),
                                expr_type));
                }
            }
            // 4. Visiting fortran allocatable symbols
            for (TL::ObjectList<TL::Symbol>::iterator it = _env.private_.begin();
                    it != _env.private_.end();
                    it++)
            {
                if (!it->is_allocatable())
                    continue;

                ERROR_CONDITION(_field_map.find(*it) == _field_map.end(), "Symbol is not mapped", 0);
                TL::Symbol field = _field_map[*it];

                names_to_fields[field.get_name()] = field;

                forwarded_parameter_names.append(field.get_name());
                forwarded_parameter_types.append(field.get_type().get_lvalue_reference_to());

                Nodecl::NodeclBase class_member_access =  Nodecl::ClassMemberAccess::make(
                        arg.make_nodecl(/* set_ref_type */ true),
                        _field_map[*it].make_nodecl(),
                        /* member_literal */ Nodecl::NodeclBase::null(),
                        _field_map[*it].get_type().get_lvalue_reference_to());
                args.append(class_member_access);

                // Finally, if the field symbol is an allocatable, we have to deallocate it.
                // Note that this copy was created when we captured its value.
                if (field.is_allocatable())
                {
                    deallocate_exprs.append(class_member_access.shallow_copy());
                }
            }

            if (_env.is_taskloop)
            {
                forwarded_parameter_names.append("taskloop_bounds");
                forwarded_parameter_types.append(taskloop_bounds_struct.get_user_defined_type().get_lvalue_reference_to());

                TL::Symbol bounds = outline_inside_scope.get_symbol_from_name("taskloop_bounds");
                args.append(bounds.make_nodecl(/* set_ref_type */ true));
            }

            TL::Symbol forwarded_function = SymbolUtils::new_function_symbol(
                    TL::Scope::get_global_scope(),
                    forwarded_name,
                    TL::Type::get_void_type(),
                    forwarded_parameter_names,
                    forwarded_parameter_types);
            // Make this symbol global
            symbol_entity_specs_set_is_static(forwarded_function.get_internal_symbol(), 0);

            Nodecl::Utils::SimpleSymbolMap forwarded_symbol_map;
            TL::ObjectList<TL::Symbol> forwarded_parameters_to_update_type;

            // We skip the first param because it's the function pointer and it's not mapped
            TL::ObjectList<TL::Symbol> forwarded_params = forwarded_function.get_related_symbols();
            for (unsigned int i = 1; i < forwarded_params.size() - 1; ++i)
            {
                TL::Symbol param(forwarded_params[i]);
                TL::Symbol field(names_to_fields[param.get_name()]);

                ERROR_CONDITION(!field.is_valid(), "Invalid symbol!", 0);

                forwarded_symbol_map.add_map(field, param);

                if (type_is_runtime_sized(param.get_type()))
                    forwarded_parameters_to_update_type.append(param);
            }

            update_function_type_if_needed(
                    forwarded_function, forwarded_parameters_to_update_type, forwarded_symbol_map);

            Nodecl::NodeclBase call_to_forward =
                Nodecl::ExpressionStatement::make(
                        Nodecl::FunctionCall::make(
                            forwarded_function.make_nodecl(/* set_ref_type */ true),
                            args,
                            /* alternate_name */ Nodecl::NodeclBase::null(),
                            /* function_form */ Nodecl::NodeclBase::null(),
                            get_void_type()));

            outline_empty_stmt.prepend_sibling(call_to_forward);


            Nodecl::List conditional_deallocation_stmts;
            for(Nodecl::List::iterator it = deallocate_exprs.begin();
                    it != deallocate_exprs.end();
                    it++)
            {
                Nodecl::NodeclBase dealloc_expr(*it);
                Nodecl::List actual_arguments = Nodecl::List::make(Nodecl::FortranActualArgument::make(dealloc_expr));

                TL::Symbol allocated = get_fortran_intrinsic_symbol<1>("allocated", actual_arguments, /* is_call */ 0);

                Nodecl::NodeclBase cond = Nodecl::FunctionCall::make(
                        allocated.make_nodecl(),
                        actual_arguments,
                        /* alternate-symbol */ Nodecl::NodeclBase::null(),
                        /* function-form */ Nodecl::NodeclBase::null(),
                        TL::Type::get_bool_type());

                Nodecl::NodeclBase dealloc_stmt = Nodecl::FortranDeallocateStatement::make(
                        Nodecl::List::make(dealloc_expr.shallow_copy()), Nodecl::NodeclBase::null());

                outline_empty_stmt.append_sibling(
                        Nodecl::IfElseStatement::make(cond, Nodecl::List::make(dealloc_stmt), Nodecl::NodeclBase::null()));
            }


            TL::ObjectList<std::string> c_forwarded_parameter_names(forwarded_parameter_names.size(), "");

            c_forwarded_parameter_names[0] = "ol";

            GenerateParamsNames generate_params_names;
            std::generate(c_forwarded_parameter_names.begin() + 1, c_forwarded_parameter_names.end(), generate_params_names);

            TL::ObjectList<TL::Type> c_forwarded_parameter_types (
                forwarded_parameter_types.size(), TL::Type::get_void_type().get_pointer_to()
                );
            // fix first
            c_forwarded_parameter_types[0] = TL::Type::get_void_type().get_function_returning(
                    TL::ObjectList<TL::Type>(forwarded_parameter_types.size() - 1, TL::Type::get_void_type().get_pointer_to())
                    );

            std::string c_forwarded_name = ::fortran_mangle_symbol(
                    forwarded_function.get_internal_symbol());

            // Now generate the C counterpart
            TL::Symbol c_forwarded_function = SymbolUtils::new_function_symbol(
                    TL::Scope::get_global_scope(),
                    c_forwarded_name,
                    TL::Type::get_void_type(),
                    c_forwarded_parameter_names,
                    c_forwarded_parameter_types);
            symbol_entity_specs_set_is_static(c_forwarded_function.get_internal_symbol(), 0);

            Nodecl::NodeclBase c_forwarded_function_code, c_forwarded_empty_stmt;
            SymbolUtils::build_empty_body_for_function(
                    c_forwarded_function,
                    c_forwarded_function_code,
                    c_forwarded_empty_stmt);

            SolveParamNames solve_param_names(c_forwarded_empty_stmt.retrieve_context());

            TL::ObjectList<Nodecl::NodeclBase> refs_to_params = c_forwarded_parameter_names.map<Nodecl::NodeclBase>(
                    solve_param_names
                    );

            Nodecl::NodeclBase c_ol_arg = refs_to_params[0];
            Nodecl::List c_args = Nodecl::List::make(
                    TL::ObjectList<Nodecl::NodeclBase>(refs_to_params.begin()+1, refs_to_params.end())
                    );

            // Note, since we are in Fortran, the empty statement must be wholly replaced by a CompoundStatement
            c_forwarded_empty_stmt.replace(
                    Nodecl::CompoundStatement::make(
                        Nodecl::List::make(
                            Nodecl::ExpressionStatement::make(
                                Nodecl::FunctionCall::make(
                                    c_ol_arg,
                                    c_args,
                                    /* alternate-symbol */ Nodecl::NodeclBase::null(),
                                    /* function-form */ Nodecl::NodeclBase::null(),
                                    TL::Type::get_void_type()))),
                        /* finalize */ Nodecl::NodeclBase::null()));

            _phase->get_extra_c_code().append(c_forwarded_function_code);
        }
        else
        {
            internal_error("Code unreachable", 0);
        }

        if (IS_CXX_LANGUAGE
                && !_related_function.is_member())
        {
            Nodecl::Utils::prepend_to_enclosing_top_level_location(_task_body,
                    Nodecl::CxxDecl::make(
                        Nodecl::Context::make(
                            Nodecl::NodeclBase::null(),
                            _related_function.get_scope()),
                        _outline_function));
        }

        Nodecl::Utils::append_to_top_level_nodecl(outline_function_code);
    }

    class ReplaceSymbolsVisitor : public Nodecl::ExhaustiveVisitor<void>
    {
        const std::map<TL::Symbol, TL::Symbol> _symbols_to_be_replaced;

        public:
        ReplaceSymbolsVisitor(const std::map<TL::Symbol, TL::Symbol>& map)
            : _symbols_to_be_replaced(map)
        { }

        void visit(const Nodecl::Symbol& node)
        {
            TL::Symbol current_symbol = node.get_symbol();
            std::map<TL::Symbol, TL::Symbol>::const_iterator it = _symbols_to_be_replaced.find(current_symbol);

            // Do nothing if the symbol is not present
            if (it == _symbols_to_be_replaced.end())
                return;

            TL::Symbol mapped_symbol(it->second);
            node.replace(mapped_symbol.make_nodecl(/* set_ref_type */ true));
        }
    };

    void TaskProperties::handle_task_reductions(
            const TL::Scope& unpacked_inside_scope,
            Nodecl::NodeclBase unpacked_empty_stmt)
    {
        for (TL::ObjectList<ReductionItem>::const_iterator it = _env.reduction.begin();
                it != _env.reduction.end();
                it++)
        {
            const ReductionItem& red_item(*it);

            TL::Symbol orig_red_var = unpacked_inside_scope.get_symbol_from_name(red_item.symbol.get_name());
            TL::Symbol priv_red_var = unpacked_inside_scope.get_symbol_from_name(red_item.symbol.get_name() + "_local_red");

            // Computing the expression statement that initializes our task local variable
            Nodecl::NodeclBase initializer = red_item.reduction_info->get_initializer().shallow_copy();
            std::map<TL::Symbol, TL::Symbol> initializer_translation_map;

            TL::Symbol omp_orig  = red_item.reduction_info->get_omp_orig();
            initializer_translation_map[omp_orig] = orig_red_var;

            ReplaceSymbolsVisitor initializer_visitor(initializer_translation_map);
            initializer_visitor.walk(initializer);

            Nodecl::NodeclBase assignment = Nodecl::Assignment::make(
                    priv_red_var.make_nodecl(/* set_ref_type */ true),
                    initializer,
                    priv_red_var.get_type().no_ref().get_lvalue_reference_to());

            unpacked_empty_stmt.prepend_sibling(Nodecl::ExpressionStatement::make(assignment));


            // Computing the combiner statements
            Nodecl::NodeclBase combiner = red_item.reduction_info->get_combiner().shallow_copy();
            std::map<TL::Symbol, TL::Symbol> combiner_translation_map;

            TL::Symbol omp_in  = red_item.reduction_info->get_omp_in();
            TL::Symbol omp_out = red_item.reduction_info->get_omp_out();
            combiner_translation_map[omp_in]  = priv_red_var;
            combiner_translation_map[omp_out] = orig_red_var;

            ReplaceSymbolsVisitor combiner_visitor(combiner_translation_map);
            combiner_visitor.walk(combiner);
            combiner = Nodecl::ExpressionStatement::make(combiner);
            unpacked_empty_stmt.append_sibling(combiner);

            TL::Scope atomic_scope = TL::Scope(
                    new_block_context(unpacked_inside_scope.get_decl_context()));

            // FIXME warning atomic -> critical should be supressed
            combiner.replace(
                    Nodecl::OpenMP::Atomic::make(
                        /* environment */ Nodecl::NodeclBase::null(),
                        Nodecl::List::make(
                            Nodecl::Context::make(
                                Nodecl::List::make(combiner.shallow_copy()),
                                atomic_scope.get_decl_context())),
                        combiner.get_locus()));

            _lower_visitor->walk(combiner);
        }
    }


    // FIXME: MOVE THIS FUNCTION TO ANOTHER PLACE
    void get_reduction_type(const TL::Type& data_type, Nodecl::NodeclBase &type_node)
    {
        std::string type_name;

        if (data_type.is_char())
            type_name = "RED_TYPE_CHAR";
        else if (data_type.is_signed_char())
            type_name = "RED_TYPE_SIGNED_CHAR";
        else if (data_type.is_unsigned_char())
            type_name = "RED_TYPE_UNSIGNED_CHAR";
        else if (data_type.is_signed_short_int())
            type_name = "RED_TYPE_SHORT";
        else if (data_type.is_unsigned_short_int())
            type_name = "RED_TYPE_UNSIGNED_SHORT";
        else if (data_type.is_signed_integral())
            type_name = "RED_TYPE_INT";
        else if (data_type.is_unsigned_integral())
            type_name = "RED_TYPE_UNSIGNED_INT";
        else if (data_type.is_signed_long_int())
            type_name = "RED_TYPE_LONG";
        else if (data_type.is_unsigned_long_int())
            type_name = "RED_TYPE_UNSIGNED_LONG";
        else if (data_type.is_signed_long_long_int())
            type_name = "RED_TYPE_LONG_LONG";
        else if (data_type.is_unsigned_long_long_int())
            type_name = "RED_TYPE_UNSIGNED_LONG_LONG";
        else if (data_type.is_float())
        {
            if (data_type.is_complex())
                type_name = "RED_TYPE_COMPLEX_FLOAT";
            else
                type_name = "RED_TYPE_FLOAT";
        }
        else if (data_type.is_double())
        {
            if (data_type.is_complex())
                type_name = "RED_TYPE_COMPLEX_DOUBLE";
            else
                type_name = "RED_TYPE_DOUBLE";
        }
        else if (data_type.is_long_double())
        {
            if (data_type.is_complex())
                type_name = "RED_TYPE_COMPLEX_LONG_DOUBLE";
            else
                type_name = "RED_TYPE_LONG_DOUBLE";
        }
        else if (data_type.is_bool())
            type_name = "RED_TYPE_BOOLEAN";

        TL::Symbol type_sym = TL::Scope::get_global_scope().get_symbol_from_name(type_name);
        ERROR_CONDITION(type_sym.is_invalid(), "'%s' symbol not found", type_name.c_str());

        type_node = type_sym.make_nodecl(/* set_ref_type */ false);
    }


    // FIXME: MOVE THIS FUNCTION TO ANOTHER PLACE
    void get_reduction_operation(const TL::OpenMP::Reduction& red_info, Nodecl::NodeclBase& operation_node)
    {
        std::string reduction_name = red_info.get_name();
        std::string operation_name;

        if (reduction_name == "+"       || reduction_name == "operator +")
            operation_name = "RED_OP_ADDITION";
        if (reduction_name == "-"       || reduction_name == "operator -")
            operation_name = "RED_OP_ADDITION";
        else if (reduction_name == "*"  || reduction_name == "operator *")
            operation_name = "RED_OP_PRODUCT";
        else if (reduction_name == "&"  || reduction_name == "operator &"  || reduction_name == "iand")
            operation_name = "RED_OP_BITWISE_AND";
        else if (reduction_name == "|"  || reduction_name == "operator |"  || reduction_name == "ior")
            operation_name = "RED_OP_BITWISE_OR";
        else if (reduction_name == "^"  || reduction_name == "operator ^"  || reduction_name == "ieor")
            operation_name = "RED_OP_BITWISE_XOR";
        else if (reduction_name == "&&" || reduction_name == "operator &&" || reduction_name == ".and.")
            operation_name = "RED_OP_LOGICAL_AND";
        else if (reduction_name == "||" || reduction_name == "operator ||" || reduction_name == ".or.")
            operation_name = "RED_OP_LOGICAL_OR";
        else if (reduction_name == ".neqv.")
            operation_name = "RED_OP_LOGICAL_XOR";
        else if (reduction_name == ".eqv.")
            operation_name = "RED_OP_LOGICAL_NXOR";
        else if (reduction_name == "max")
            operation_name = "RED_OP_MAXIMUM";
        else if (reduction_name == "min")
            operation_name = "RED_OP_MINIMUM";

        TL::Symbol operation_sym = TL::Scope::get_global_scope().get_symbol_from_name(operation_name);
        ERROR_CONDITION(operation_sym.is_invalid(), "'%s' symbol not found", operation_name.c_str());

        operation_node = operation_sym.make_nodecl(/* set_ref_type */ false);
    }

    void TaskProperties::compute_reduction_arguments_register_dependence(
            TL::DataReference& data_ref,
            // Out
            TL::ObjectList<Nodecl::NodeclBase>& arguments_list)
    {
        // 1st argument: type operation identifier
        TL::ObjectList<ReductionItem> red_items =
            _env.reduction.find<TL::Symbol>(&ReductionItem::get_symbol, data_ref.get_base_symbol());

        ERROR_CONDITION(red_items.empty(), "No reduction item for symbol '%s'",
                data_ref.get_base_symbol().get_name().c_str());

        TL::OpenMP::Reduction *reduction_info = red_items.begin()->reduction_info;

        Nodecl::NodeclBase type_node;
        Nodecl::NodeclBase operation_node;
        get_reduction_type(reduction_info->get_type(), type_node);
        get_reduction_operation(*reduction_info, operation_node);

        Nodecl::NodeclBase arg1_type_op = Nodecl::Add::make(
                type_node,
                operation_node,
                reduction_info->get_type()
                );

        // 2nd argument: reduction identifier within task
        Nodecl::NodeclBase arg2_id = const_value_to_nodecl(
                const_value_get_unsigned_int(_num_reductions));

        // Increment number of registered reductions for the task (used as id when registering)
        _num_reductions++;

        arguments_list.append(arg1_type_op);
        arguments_list.append(arg2_id);
    }


    namespace {

    void compute_dimensionality_information_c(
            TL::Type type,
            // Out
            TL::ObjectList<Nodecl::NodeclBase>& arguments_list)
    {
        Nodecl::NodeclBase size, lower_bound, upper_bound;
        if (type.is_array())
        {
            TL::Type element_type = type.array_element();
            if (element_type.is_array())
                compute_dimensionality_information_c(element_type, arguments_list);

            size = type.array_get_size().shallow_copy();

            if (type.array_is_region())
                type.array_get_region_bounds(lower_bound, upper_bound);
            else
                type.array_get_bounds(lower_bound, upper_bound);

            lower_bound = lower_bound.shallow_copy();
            upper_bound =
                Nodecl::Add::make(
                        upper_bound.shallow_copy(),
                        const_value_to_nodecl(const_value_get_one(8, 0)),
                        upper_bound.get_type().no_ref());

            // Continuous dimension should be expressed in bytes
            if (!element_type.is_array())
            {
                Nodecl::NodeclBase element_type_size = Nodecl::Sizeof::make(
                        Nodecl::Type::make(element_type),
                        Nodecl::NodeclBase::null(),
                        get_size_t_type());

                size = Nodecl::Mul::make(size, element_type_size, size.get_type().no_ref());
                lower_bound = Nodecl::Mul::make(
                        lower_bound,
                        element_type_size.shallow_copy(),
                        lower_bound.get_type().no_ref());

                upper_bound = Nodecl::Mul::make(
                        upper_bound,
                        element_type_size.shallow_copy(),
                        upper_bound.get_type().no_ref());
            }
        }
        else
        {
            // Continuous dimension should be expressed in bytes
            size = Nodecl::Sizeof::make(
                    Nodecl::Type::make(type),
                    Nodecl::NodeclBase::null(),
                    get_size_t_type());

            lower_bound = const_value_to_nodecl(const_value_get_zero(8, 0));
            upper_bound = size.shallow_copy();
        }

        arguments_list.append(size);
        arguments_list.append(lower_bound);
        arguments_list.append(upper_bound);
    }

    void compute_dimensionality_information_fortran(
            const TL::DataReference& data_ref,
            TL::Type type,
            // Out
            TL::ObjectList<Nodecl::NodeclBase>& arguments_list)
    {
        Nodecl::NodeclBase size, lower_bound, upper_bound;
        if (type.is_array())
        {
            TL::Type element_type = type.array_element();
            if (element_type.is_array())
                compute_dimensionality_information_fortran(data_ref, element_type, arguments_list);

            Nodecl::NodeclBase array_lb, array_ub;
            {
                type.array_get_bounds(array_lb, array_ub);
                if (array_lb.is_null())
                    array_lb = TL::Lowering::Utils::Fortran::get_lower_bound(data_ref, type.fortran_rank());
                else
                    array_lb = array_lb.shallow_copy();

                if (array_ub.is_null())
                    array_ub = TL::Lowering::Utils::Fortran::get_upper_bound(data_ref, type.fortran_rank());
                else
                    array_ub = array_ub.shallow_copy();
            }

            Nodecl::NodeclBase region_lb, region_ub;
            {
                if (type.array_is_region())
                {
                    type.array_get_region_bounds(region_lb, region_ub);
                }
                else
                {
                    region_lb = array_lb.shallow_copy();
                    region_ub = array_ub.shallow_copy();
                }
            }

            size = TL::Lowering::Utils::Fortran::get_size_for_dimension(data_ref, type, type.fortran_rank());

            lower_bound = Nodecl::Minus::make(
                    region_lb,
                    array_lb,
                    region_lb.get_type().no_ref());

            //XXX: ADD one?
            upper_bound = Nodecl::Add::make(
                    Nodecl::Minus::make(
                        region_ub,
                        array_lb,
                        region_lb.get_type().no_ref()),
                    const_value_to_nodecl(const_value_get_one(8, 0)),
                    region_lb.get_type().no_ref());

            // Continuous dimension should be expressed in bytes
            if (!element_type.is_array())
            {
                Nodecl::NodeclBase element_type_size = Nodecl::Sizeof::make(
                        Nodecl::Type::make(element_type),
                        Nodecl::NodeclBase::null(),
                        get_size_t_type());

                size = Nodecl::Mul::make(
                        Nodecl::ParenthesizedExpression::make(size, size.get_type().no_ref()),
                        element_type_size,
                        size.get_type().no_ref());

                lower_bound = Nodecl::Mul::make(
                        Nodecl::ParenthesizedExpression::make(lower_bound, lower_bound.get_type().no_ref()),
                        element_type_size.shallow_copy(),
                        lower_bound.get_type().no_ref());

                upper_bound = Nodecl::Mul::make(
                        Nodecl::ParenthesizedExpression::make(upper_bound, upper_bound.get_type().no_ref()),
                        element_type_size.shallow_copy(),
                        upper_bound.get_type().no_ref());
            }
        }
        else
        {
            size = data_ref.get_sizeof().shallow_copy();
            lower_bound = const_value_to_nodecl(const_value_get_zero(8, 0));
            upper_bound = size.shallow_copy();
        }

        // Fortran is a bit repellent checking the actual arguments types, for
        // this reason we may need to add some conversions
        TL::Type param_type = fortran_choose_int_type_from_kind(8);
        arguments_list.append(Nodecl::Conversion::make(size, param_type));
        arguments_list.append(Nodecl::Conversion::make(lower_bound, param_type));
        arguments_list.append(Nodecl::Conversion::make(upper_bound, param_type));
    }

    void compute_base_address_and_dimensionality_information(
            TL::DataReference& data_ref,
            // Out
            TL::ObjectList<Nodecl::NodeclBase>& arguments_list)
    {
        Nodecl::NodeclBase base_address =
            Nodecl::Conversion::make(
                data_ref.get_base_address().shallow_copy(),
                TL::Type::get_void_type().get_pointer_to());

        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
            base_address.set_text("C");

        arguments_list.append(base_address);

        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
            compute_dimensionality_information_c(data_ref.get_data_type(), arguments_list);
        else
            compute_dimensionality_information_fortran(data_ref, data_ref.get_data_type(), arguments_list);
    }

    void compute_arguments_register_dependence(
            TL::DataReference& data_ref,
            TL::Symbol handler,
            // Out
            TL::ObjectList<Nodecl::NodeclBase>& arguments_list)
    {
        // task handler
        arguments_list.append(handler.make_nodecl(/* set_ref_type */ true));

        // sym identifier
        arguments_list.append(const_value_to_nodecl(const_value_get_minus_one(4, 1)));

        // dependence text
        std::string dependence_text = Codegen::get_current().codegen_to_str(data_ref, data_ref.retrieve_context());
        arguments_list.append(const_value_to_nodecl(
                const_value_make_string_null_ended(
                    dependence_text.c_str(),
                    strlen(dependence_text.c_str()))));

        compute_base_address_and_dimensionality_information(data_ref, arguments_list);
    }

    }

    void TaskProperties::register_dependence_c(
            TL::DataReference& data_ref,
            TL::Symbol handler,
            TL::Symbol arg,
            TL::Symbol register_fun,
            const TL::ObjectList<TL::Symbol>& local_symbols,
            // Out
            Nodecl::List& register_statements)
    {
        std::string reduction_register_fun_name = "nanos_register_region_reduction_depinfo";
        bool is_reduction = reduction_register_fun_name.compare(
                0, std::string::npos, register_fun.get_name(),
                0, reduction_register_fun_name.length()) == 0;

        TL::ObjectList<Nodecl::NodeclBase> arguments;
        if (is_reduction)
        {
            ERROR_CONDITION(data_ref.get_data_type().is_array(), "Array reductions not supported", 0);
            compute_reduction_arguments_register_dependence(data_ref, arguments);
        }
        compute_arguments_register_dependence(data_ref, handler, arguments);

        Nodecl::List args;
        for(TL::ObjectList<Nodecl::NodeclBase>::iterator it = arguments.begin();
                it != arguments.end();
                it++)
        {
            args.append(rewrite_expression_using_args(arg, *it, local_symbols));
        }

        Nodecl::NodeclBase function_call =
            Nodecl::ExpressionStatement::make(
                    Nodecl::FunctionCall::make(
                        register_fun.make_nodecl(/* set_ref_type */ true),
                        args,
                        /* alternate_name */ Nodecl::NodeclBase::null(),
                        /* function_form */ Nodecl::NodeclBase::null(),
                        get_void_type()));

        register_statements.append(function_call);
    }


    void TaskProperties::register_multidependence_c(
            TL::DataReference& data_ref,
            TL::Symbol handler,
            TL::Symbol arg,
            TL::Symbol register_fun,
            TL::Scope scope,
            // Out
            TL::ObjectList<TL::Symbol>& local_symbols,
            Nodecl::List& register_statements)
    {
        TL::ObjectList<TL::DataReference::MultiRefIterator> multireferences = data_ref.multireferences();

        Nodecl::Utils::SimpleSymbolMap symbol_map;
        TL::Counter &ctr = TL::CounterManager::get_counter("nanos6-multideps");
        for (TL::ObjectList<TL::DataReference::MultiRefIterator>::iterator it2 = multireferences.begin();
                it2 != multireferences.end();
                it2++)
        {
            std::stringstream ss;
            ss << it2->first.get_name() << "_tmp_" << (int)ctr;
            ctr++;
            std::string ind_var_name = ss.str();

            TL::Symbol local_sym = scope.new_symbol(ind_var_name);
            local_sym.get_internal_symbol()->kind = SK_VARIABLE;
            local_sym.get_internal_symbol()->type_information =
                ::get_signed_int_type();
            symbol_entity_specs_set_is_user_declared(local_sym.get_internal_symbol(),
                    1);

            symbol_map.add_map(it2->first, local_sym);
            local_symbols.append(local_sym);

            CXX_LANGUAGE()
            {
                register_statements.append(
                        Nodecl::CxxDef::make(Nodecl::NodeclBase::null(),
                            local_sym));
            }
        }


        // This empty stmt will be replaced by the function call to the runtime
        Nodecl::NodeclBase empty_stmt = Nodecl::EmptyStatement::make();

        Nodecl::NodeclBase body = Nodecl::List::make(empty_stmt);
        for (TL::ObjectList<TL::DataReference::MultiRefIterator>::reverse_iterator it2 = multireferences.rbegin();
                it2 != multireferences.rend();
                it2++)
        {
            ERROR_CONDITION(!it2->second.is<Nodecl::Range>(), "Invalid Node", 0);

            Nodecl::Range range = it2->second.as<Nodecl::Range>();
            Nodecl::NodeclBase lower = rewrite_expression_using_args(arg, range.get_lower(), local_symbols);
            Nodecl::NodeclBase upper = rewrite_expression_using_args(arg, range.get_upper(), local_symbols);
            Nodecl::NodeclBase stride = rewrite_expression_using_args(arg, range.get_stride(), local_symbols);

            TL::Symbol ind_var = symbol_map.map(it2->first);

            Nodecl::NodeclBase loop_control =
                Nodecl::LoopControl::make(
                    // init-expr-list
                        Nodecl::List::make(
                            Nodecl::Assignment::make(
                                ind_var.make_nodecl(/* set_ref_type */ true),
                                lower,
                                ind_var.get_type().no_ref().get_lvalue_reference_to())),
                    // test-expr
                    Nodecl::LowerOrEqualThan::make(
                        ind_var.make_nodecl(/* set_ref_type */ true),
                        upper,
                        TL::Type::get_bool_type()),
                    // incr-exp
                    Nodecl::AddAssignment::make(
                        ind_var.make_nodecl(/* set_ref_type */ true),
                        stride,
                        ind_var.get_type().no_ref().get_lvalue_reference_to()));

            // Note that we are not creating any context / compound stmt. Thus, the body has to be always an statement
            Nodecl::NodeclBase for_stmt =
                Nodecl::ForStatement::make(loop_control, body, /* loop-name */ Nodecl::NodeclBase::null());

            body = Nodecl::List::make(for_stmt);
        }

        Nodecl::NodeclBase base_exp = data_ref;
        while (base_exp.is<Nodecl::MultiExpression>())
            base_exp = base_exp.as<Nodecl::MultiExpression>().get_base();

        base_exp = Nodecl::Utils::deep_copy(base_exp, scope, symbol_map);

        TL::DataReference base_data_ref = base_exp;
        Nodecl::List base_reg;

        register_dependence_c(
                base_data_ref,
                handler,
                arg,
                register_fun,
                local_symbols,
                base_reg);

        empty_stmt.replace(base_reg);

        register_statements.append(body);
    }

    void TaskProperties::create_dependences_function_c()
    {
        TL::ObjectList<std::string> dep_parameter_names(2);
        dep_parameter_names[0] = "handler";
        dep_parameter_names[1] = "arg";
        TL::ObjectList<TL::Type> dep_parameter_types(2);
        dep_parameter_types[0] = TL::Type::get_void_type().get_pointer_to();
        dep_parameter_types[1] = _info_structure.get_lvalue_reference_to();

        std::string dep_name = get_new_name("nanos6_dep");

        _dependences_function
            = SymbolUtils::new_function_symbol(
                    _related_function,
                    dep_name,
                    TL::Type::get_void_type(),
                    dep_parameter_names,
                    dep_parameter_types);

        Nodecl::NodeclBase dependences_function_code, dependences_empty_stmt;
        SymbolUtils::build_empty_body_for_function(
                _dependences_function,
                dependences_function_code,
                dependences_empty_stmt);

        TL::Scope dependences_inside_scope = dependences_empty_stmt.retrieve_context();
        TL::Symbol handler = dependences_inside_scope.get_symbol_from_name("handler");
        ERROR_CONDITION(!handler.is_valid(), "Invalid symbol", 0);
        TL::Symbol arg = dependences_inside_scope.get_symbol_from_name("arg");
        ERROR_CONDITION(!arg.is_valid(), "Invalid symbol", 0);

        TL::Scope global_context = TL::Scope::get_global_scope();


        struct DependencesSet
        {
            TL::ObjectList<Nodecl::NodeclBase> &dep_list;
            std::string func_name;
        } deps[] = {
            { _env.dep_in,    "nanos_register_region_read_depinfo"              },
            { _env.dep_out,   "nanos_register_region_write_depinfo"             },
            { _env.dep_inout, "nanos_register_region_readwrite_depinfo"         },

            { _env.dep_weakin,    "nanos_register_region_weak_read_depinfo"      },
            { _env.dep_weakout,   "nanos_register_region_weak_write_depinfo"     },
            { _env.dep_weakinout, "nanos_register_region_weak_readwrite_depinfo" },

            { _env.dep_commutative, "nanos_register_region_commutative_depinfo" },
            { _env.dep_concurrent,  "nanos_register_region_concurrent_depinfo"  },

            { _env.dep_reduction, "nanos_register_region_reduction_depinfo" },
        };


        for (DependencesSet *dep_set = deps;
                dep_set != (DependencesSet*)(&deps + 1);
                dep_set++)
        {
            TL::ObjectList<Nodecl::NodeclBase> &dep_list = dep_set->dep_list;
            if (dep_list.empty())
                continue;

            for (TL::ObjectList<Nodecl::NodeclBase>::iterator
                    it = dep_list.begin();
                    it != dep_list.end();
                    it++)
            {
                TL::DataReference data_ref = *it;
                TL::Type data_type = data_ref.get_data_type();

                TL::Symbol register_fun;
                {
                    int max_dimensions = _phase->nanos6_api_max_dimensions();
                    ERROR_CONDITION(data_type.is_array() &&
                            (data_type.get_num_dimensions() > max_dimensions),
                            "Maximum number of data dimensions allowed is %d",
                            max_dimensions);

                    int num_dims_dep = data_type.is_array() ? data_type.get_num_dimensions() : 1;
                    std::stringstream ss;
                    ss << dep_set->func_name << num_dims_dep;

                    register_fun = global_context.get_symbol_from_name(ss.str());
                    if (!register_fun.is_valid())
                    {
                        fatal_error(
                                "'%s' function not found while trying to register dependences\n",
                                ss.str().c_str());
                    }
                }

                TL::ObjectList<TL::Symbol> local_symbols;
                local_symbols.append(handler);

                Nodecl::List register_statements;
                if (!data_ref.is_multireference())
                {
                    register_dependence_c(
                            data_ref,
                            handler,
                            arg,
                            register_fun,
                            local_symbols,
                            register_statements);
                }
                else
                {
                    register_multidependence_c(
                            data_ref,
                            handler,
                            arg,
                            register_fun,
                            dependences_inside_scope,
                            local_symbols,
                            register_statements);
                }
                dependences_empty_stmt.prepend_sibling(register_statements);
            }
        }

        if (IS_CXX_LANGUAGE
                && !_related_function.is_member())
        {
            Nodecl::Utils::prepend_to_enclosing_top_level_location(_task_body,
                    Nodecl::CxxDecl::make(
                        Nodecl::Context::make(
                            Nodecl::NodeclBase::null(),
                            _related_function.get_scope()),
                        _dependences_function));
        }

        Nodecl::Utils::append_to_enclosing_top_level_location(_task_body, dependences_function_code);
    }

    void TaskProperties::register_dependence_fortran(
        TL::DataReference &data_ref,
        TL::Symbol handler,
        Nodecl::Utils::SymbolMap &symbol_map,
        TL::Symbol register_fun,
        Nodecl::List &register_statements)
    {
        std::string reduction_register_fun_name = "nanos_register_region_reduction_depinfo";
        bool is_reduction = reduction_register_fun_name.compare(
                0, std::string::npos, register_fun.get_name(),
                0, reduction_register_fun_name.length()) == 0;

        TL::ObjectList<Nodecl::NodeclBase> arguments_list;
        if (is_reduction)
        {
            ERROR_CONDITION(data_ref.get_data_type().is_array(), "Array reductions not supported", 0);
            compute_reduction_arguments_register_dependence(data_ref, arguments_list);
        }
        compute_arguments_register_dependence(data_ref, handler, arguments_list);

        Nodecl::List args = Nodecl::List::make(arguments_list);

        Nodecl::NodeclBase function_call
            = Nodecl::ExpressionStatement::make(Nodecl::FunctionCall::make(
                register_fun.make_nodecl(/* set_ref_type */ true),
                args,
                /* alternate-symbol */ Nodecl::NodeclBase::null(),
                /* function-form */ Nodecl::NodeclBase::null(),
                TL::Type::get_void_type()));

        register_statements.append(Nodecl::Utils::deep_copy(function_call, TL::Scope::get_global_scope(), symbol_map));
    }

    void TaskProperties::register_multidependence_fortran(
        TL::DataReference &data_ref,
        TL::Symbol handler,
        Nodecl::Utils::SymbolMap &symbol_map,
        TL::Symbol register_fun,
        TL::Scope scope,
        // Out
        Nodecl::List &register_statements)
    {
        TL::ObjectList<TL::DataReference::MultiRefIterator> multireferences = data_ref.multireferences();

        Nodecl::Utils::SimpleSymbolMap extended_symbol_map(&symbol_map);
        TL::ObjectList<TL::Symbol> current_locals;
        TL::Counter &ctr = TL::CounterManager::get_counter("nanos6-multideps");
        for (TL::ObjectList<TL::DataReference::MultiRefIterator>::iterator it2 = multireferences.begin();
                it2 != multireferences.end();
                it2++)
        {
            std::stringstream ss;
            ss << it2->first.get_name() << "_tmp_" << (int)ctr;
            ctr++;
            std::string ind_var_name = ss.str();

            TL::Symbol local_sym = scope.new_symbol(ind_var_name);
            local_sym.get_internal_symbol()->kind = SK_VARIABLE;
            local_sym.get_internal_symbol()->type_information =
                ::get_signed_int_type();
            symbol_entity_specs_set_is_user_declared(local_sym.get_internal_symbol(), 1);

            extended_symbol_map.add_map(it2->first, local_sym);
            current_locals.append(local_sym);

            CXX_LANGUAGE()
            {
                register_statements.append(
                        Nodecl::CxxDef::make(Nodecl::NodeclBase::null(),
                            local_sym));
            }
        }

        Nodecl::NodeclBase empty_stmt = Nodecl::EmptyStatement::make();
        Nodecl::NodeclBase body = Nodecl::List::make(empty_stmt);

        for (TL::ObjectList<TL::DataReference::MultiRefIterator>::reverse_iterator it2 = multireferences.rbegin();
                it2 != multireferences.rend();
                it2++)
        {
            ERROR_CONDITION(!it2->second.is<Nodecl::Range>(), "Invalid Node", 0);
            Nodecl::Range range = it2->second.as<Nodecl::Range>();
            Nodecl::NodeclBase lower = range.get_lower().shallow_copy();
            Nodecl::NodeclBase upper = range.get_upper().shallow_copy();
            Nodecl::NodeclBase stride = range.get_stride().shallow_copy();

            TL::Symbol ind_var = extended_symbol_map.map(it2->first);

            Nodecl::NodeclBase loop_control =
                Nodecl::RangeLoopControl::make(
                        // ind-var
                        ind_var.make_nodecl(/* set_ref_type */ true),
                        lower,
                        upper,
                        stride);

            Nodecl::NodeclBase for_stmt =
                Nodecl::ForStatement::make(loop_control, body, /* loop-name */ Nodecl::NodeclBase::null());
            body = Nodecl::List::make(for_stmt);
        }

        Nodecl::NodeclBase base_exp = data_ref;
        while (base_exp.is<Nodecl::MultiExpression>())
            base_exp = base_exp.as<Nodecl::MultiExpression>().get_base();

        TL::DataReference base_data_ref = base_exp;
        Nodecl::List base_reg;

        register_dependence_fortran(
                base_data_ref,
                handler,
                extended_symbol_map,
                register_fun,
                // Out
                base_reg);

        empty_stmt.replace(base_reg);

        register_statements.append(body);
    }


    void TaskProperties::create_dependences_function_fortran_proper()
    {
        // This is similar to the dep_fun function
        TL::ObjectList<std::string> dep_fun_param_names;
        TL::ObjectList<TL::Type> dep_fun_param_types;
        std::map<TL::Symbol, std::string> symbols_to_param_names;

        std::string dep_fun_name = get_new_name("nanos6_unpack_dep");

        dep_fun_param_names.append("handler");
        dep_fun_param_types.append(TL::Type::get_void_type().get_pointer_to());

        AddParameter add_params_functor(
                /* out */ dep_fun_param_names,
                /* out */ dep_fun_param_types,
                /* out */ symbols_to_param_names,
                /* map_reduction_local_symbol */ false);

        _env.captured_value.map(add_params_functor);
        _env.shared.map(add_params_functor);
        _env.reduction.map(add_params_functor);

        _dependences_function
            = SymbolUtils::new_function_symbol(_related_function,
                                               dep_fun_name,
                                               TL::Type::get_void_type(),
                                               dep_fun_param_names,
                                               dep_fun_param_types);

        Nodecl::NodeclBase dep_fun_function_code, dep_fun_empty_stmt;
        SymbolUtils::build_empty_body_for_function(
            _dependences_function, dep_fun_function_code, dep_fun_empty_stmt);

        TL::Scope dep_fun_inside_scope = _dependences_function.get_related_scope();

        fortran_add_types(dep_fun_inside_scope);

        // Prepare deep copy and remember those parameters that need fixup
        Nodecl::Utils::SimpleSymbolMap symbol_map;
        TL::ObjectList<TL::Symbol> parameters_to_update_type;

        MapSymbols map_symbols_functor(
                dep_fun_inside_scope,
                symbols_to_param_names,
                /* out */ symbol_map,
                /* out */ parameters_to_update_type);

        _env.captured_value.map(map_symbols_functor);
        _env.shared.map(map_symbols_functor);
        _env.reduction.map(map_symbols_functor);

        update_function_type_if_needed(
                _dependences_function, parameters_to_update_type, symbol_map);

        TL::Symbol handler
            = dep_fun_inside_scope.get_symbol_from_name("handler");
        ERROR_CONDITION(!handler.is_valid(), "Invalid symbol", 0);

        TL::Scope global_context = TL::Scope::get_global_scope();

        struct DependencesSet
        {
            TL::ObjectList<Nodecl::NodeclBase> &dep_list;
            std::string func_name;
        } deps[] = {
            { _env.dep_in,    "nanos_register_region_read_depinfo"              },
            { _env.dep_out,   "nanos_register_region_write_depinfo"             },
            { _env.dep_inout, "nanos_register_region_readwrite_depinfo"         },

            { _env.dep_weakin,    "nanos_register_region_weak_read_depinfo"      },
            { _env.dep_weakout,   "nanos_register_region_weak_write_depinfo"     },
            { _env.dep_weakinout, "nanos_register_region_weak_readwrite_depinfo" },

            { _env.dep_commutative, "nanos_register_region_commutative_depinfo" },
            { _env.dep_concurrent,  "nanos_register_region_concurrent_depinfo"  },

            { _env.dep_reduction, "nanos_register_region_reduction_depinfo" },
        };

        for (DependencesSet *dep_set = deps;
             dep_set != (DependencesSet *)(&deps + 1);
             dep_set++)
        {
            TL::ObjectList<Nodecl::NodeclBase> &dep_list = dep_set->dep_list;

            if (dep_list.empty())
                continue;

            for (TL::ObjectList<Nodecl::NodeclBase>::iterator it
                 = dep_list.begin();
                 it != dep_list.end();
                 it++)
            {
                TL::DataReference data_ref = *it;
                TL::Type data_type = data_ref.get_data_type();

                TL::Symbol register_fun;
                {
                    int max_dimensions = _phase->nanos6_api_max_dimensions();
                    ERROR_CONDITION(data_type.is_array() &&
                            (data_type.get_num_dimensions() > max_dimensions),
                            "Maximum number of data dimensions allowed is %d",
                            max_dimensions);

                    int num_dims_dep = data_type.is_array() ? data_type.get_num_dimensions() : 1;
                    std::stringstream ss;
                    ss << dep_set->func_name << num_dims_dep;

                    register_fun = global_context.get_symbol_from_name(ss.str());
                    if (!register_fun.is_valid())
                    {
                        fatal_error(
                                "'%s' function not found while trying to register dependences\n",
                                ss.str().c_str());
                    }
                }

                Nodecl::List register_statements;
                if (!data_ref.is_multireference())
                {
                    register_dependence_fortran(
                            data_ref,
                            handler,
                            symbol_map,
                            register_fun,
                            // Out
                            register_statements);
                }
                else
                {
                    register_multidependence_fortran(
                            data_ref,
                            handler,
                            symbol_map,
                            register_fun,
                            dep_fun_inside_scope,
                            // Out
                            register_statements);
                }

                dep_fun_empty_stmt.prepend_sibling(register_statements);
            }
        }

        Nodecl::Utils::append_to_enclosing_top_level_location(
            _task_body, dep_fun_function_code);
    }

    void TaskProperties::create_dependences_function_fortran_forward()
    {
        /* ===================== */
        /* Dependences function that simply calls the forward function */
        /* ===================== */
        TL::ObjectList<std::string> dep_parameter_names(2);
        dep_parameter_names[0] = "handler";
        dep_parameter_names[1] = "arg";
        TL::ObjectList<TL::Type> dep_parameter_types(2);
        dep_parameter_types[0] = TL::Type::get_void_type().get_pointer_to();
        dep_parameter_types[1] = _info_structure.get_lvalue_reference_to();

        std::string dep_name = get_new_name("nanos6_ol_dep");

        TL::Symbol proper_dependences_function = _dependences_function;

        _dependences_function
            = SymbolUtils::new_function_symbol(_related_function,
                                               dep_name,
                                               TL::Type::get_void_type(),
                                               dep_parameter_names,
                                               dep_parameter_types);

        Nodecl::NodeclBase dependences_function_code, dependences_empty_stmt;
        SymbolUtils::build_empty_body_for_function(_dependences_function,
                                                   dependences_function_code,
                                                   dependences_empty_stmt);

        TL::Scope dependences_inside_scope
            = dependences_empty_stmt.retrieve_context();
        TL::Symbol handler
            = dependences_inside_scope.get_symbol_from_name("handler");
        ERROR_CONDITION(!handler.is_valid(), "Invalid symbol", 0);
        TL::Symbol arg = dependences_inside_scope.get_symbol_from_name("arg");
        ERROR_CONDITION(!arg.is_valid(), "Invalid symbol", 0);

        Nodecl::Utils::append_to_enclosing_top_level_location(
            _task_body, dependences_function_code);

        fortran_add_types(dependences_inside_scope);


        /* ===================== */
        /* Dependences function that simply calls the forward function */
        /* Fortran side */
        /* ===================== */
        std::string forwarded_name = get_new_name("nanos6_fwd_dep");

        TL::ObjectList<std::string> forwarded_parameter_names;
        TL::ObjectList<TL::Type> forwarded_parameter_types;

        forwarded_parameter_names.append("ol");
        forwarded_parameter_types.append(
            TL::Type::get_void_type().get_pointer_to());
        forwarded_parameter_names.append("handler");
        forwarded_parameter_types.append(
            TL::Type::get_void_type().get_pointer_to());

        Nodecl::List args;

        // ol
        args.append(Nodecl::Reference::make(
            proper_dependences_function.make_nodecl(/* set_ref_type */ true),
            proper_dependences_function.get_type().get_pointer_to()));
        // handler
        args.append(handler.make_nodecl(/* set_ref_type */ true));

        for (TL::ObjectList<TL::Symbol>::iterator it = _env.captured_value.begin();
             it != _env.captured_value.end();
             it++)
        {
            ERROR_CONDITION(_field_map.find(*it) == _field_map.end(),
                            "Symbol is not mapped",
                            0);
            TL::Symbol field = _field_map[*it];

            forwarded_parameter_names.append(field.get_name());
            forwarded_parameter_types.append(
                field.get_type().get_lvalue_reference_to());

            args.append(Nodecl::ClassMemberAccess::make(
                // Nodecl::Dereference::make(
                arg.make_nodecl(/* set_ref_type */ true),
                //    arg.get_type().points_to().get_lvalue_reference_to()),
                _field_map[*it].make_nodecl(),
                /* member_literal */ Nodecl::NodeclBase::null(),
                _field_map[*it].get_type().get_lvalue_reference_to()));
        }

        for (TL::ObjectList<TL::Symbol>::iterator it = _env.shared.begin();
             it != _env.shared.end();
             it++)
        {
            ERROR_CONDITION(_field_map.find(*it) == _field_map.end(),
                            "Symbol is not mapped",
                            0);
            TL::Symbol field = _field_map[*it];

            forwarded_parameter_names.append(field.get_name());
            forwarded_parameter_types.append(field.get_type());

            args.append(Nodecl::ClassMemberAccess::make(
                // Nodecl::Dereference::make(
                arg.make_nodecl(/* set_ref_type */ true),
                //    arg.get_type().points_to().get_lvalue_reference_to()),
                _field_map[*it].make_nodecl(),
                /* member_literal */ Nodecl::NodeclBase::null(),
                _field_map[*it].get_type().get_lvalue_reference_to()));
        }

        for (TL::ObjectList<ReductionItem>::const_iterator it = _env.reduction.begin();
                it != _env.reduction.end();
                it++)
        {
            ERROR_CONDITION(_field_map.find(it->symbol) == _field_map.end(),
                    "Symbol is not mapped", 0);

            // First, original variable
            {
                TL::Symbol field = _field_map[it->symbol];

                forwarded_parameter_names.append(field.get_name());
                forwarded_parameter_types.append(field.get_type());

                args.append(
                        Nodecl::ClassMemberAccess::make(
                            arg.make_nodecl(/* set_ref_type */ true),
                            _field_map[it->symbol].make_nodecl(),
                            /* member_literal */ Nodecl::NodeclBase::null(),
                            _field_map[it->symbol].get_type().no_ref().get_lvalue_reference_to()));
            }

            // Second, local variable
            {
                TL::Scope inner_class_context(
                        class_type_get_inner_context(_info_structure.get_internal_type()));

                TL::Symbol local_symbol = inner_class_context.get_symbol_from_name(it->symbol.get_name() + "_local_red");
                TL::Type expr_type = local_symbol.get_type().no_ref().get_lvalue_reference_to();

                forwarded_parameter_names.append(local_symbol.get_name());
                forwarded_parameter_types.append(expr_type);

                args.append(
                        Nodecl::ClassMemberAccess::make(
                            arg.make_nodecl(/* set_ref_type */ true),
                            local_symbol.make_nodecl(),
                            /* member_literal */ Nodecl::NodeclBase::null(),
                            expr_type));
            }
        }

        TL::Symbol forwarded_function
            = SymbolUtils::new_function_symbol(TL::Scope::get_global_scope(),
                                               forwarded_name,
                                               TL::Type::get_void_type(),
                                               forwarded_parameter_names,
                                               forwarded_parameter_types);
        // Make this symbol global
        symbol_entity_specs_set_is_static(
            forwarded_function.get_internal_symbol(), 0);

        Nodecl::NodeclBase call_to_forward
            = Nodecl::ExpressionStatement::make(Nodecl::FunctionCall::make(
                forwarded_function.make_nodecl(/* set_ref_type */ true),
                args,
                /* alternate_name */ Nodecl::NodeclBase::null(),
                /* function_form */ Nodecl::NodeclBase::null(),
                get_void_type()));

        dependences_empty_stmt.prepend_sibling(call_to_forward);

        /* ===================== */
        /* Dependences function that simply calls the forward function */
        /* C side */
        /* ===================== */
        std::string c_forwarded_name
            = ::fortran_mangle_symbol(forwarded_function.get_internal_symbol());

        // Now generate the C counterpart
        TL::ObjectList<std::string> c_forwarded_parameter_names(
            forwarded_parameter_names.size(), "");

        c_forwarded_parameter_names[0] = "ol";

        GenerateParamsNames generate_params_names;
        std::generate(c_forwarded_parameter_names.begin() + 1,
                      c_forwarded_parameter_names.end(),
                      generate_params_names);

        TL::ObjectList<TL::Type> c_forwarded_parameter_types(
            forwarded_parameter_types.size(),
            TL::Type::get_void_type().get_pointer_to());
        c_forwarded_parameter_types[0]
            = TL::Type::get_void_type().get_function_returning(
                TL::ObjectList<TL::Type>(
                    forwarded_parameter_types.size() - 1,
                    TL::Type::get_void_type().get_pointer_to()));

        TL::Symbol c_forwarded_function
            = SymbolUtils::new_function_symbol(TL::Scope::get_global_scope(),
                                               c_forwarded_name,
                                               TL::Type::get_void_type(),
                                               c_forwarded_parameter_names,
                                               c_forwarded_parameter_types);
        symbol_entity_specs_set_is_static(
            c_forwarded_function.get_internal_symbol(), 0);

        Nodecl::NodeclBase c_forwarded_function_code, c_forwarded_empty_stmt;
        SymbolUtils::build_empty_body_for_function(c_forwarded_function,
                                                   c_forwarded_function_code,
                                                   c_forwarded_empty_stmt);

        SolveParamNames solve_param_names(
            c_forwarded_empty_stmt.retrieve_context());

        TL::ObjectList<Nodecl::NodeclBase> refs_to_params
            = c_forwarded_parameter_names.map<Nodecl::NodeclBase>(
                solve_param_names);

        Nodecl::NodeclBase c_ol_arg = refs_to_params[0];
        Nodecl::List c_args
            = Nodecl::List::make(TL::ObjectList<Nodecl::NodeclBase>(
                refs_to_params.begin() + 1, refs_to_params.end()));

        c_forwarded_empty_stmt.replace(Nodecl::CompoundStatement::make(
            Nodecl::List::make(
                Nodecl::ExpressionStatement::make(Nodecl::FunctionCall::make(
                    c_ol_arg,
                    c_args,
                    /* alternate-symbol */ Nodecl::NodeclBase::null(),
                    /* function-form */ Nodecl::NodeclBase::null(),
                    TL::Type::get_void_type()))),
            /* finalize */ Nodecl::NodeclBase::null()));

        _phase->get_extra_c_code().append(c_forwarded_function_code);
    }

    void TaskProperties::create_dependences_function_fortran()
    {
        create_dependences_function_fortran_proper();
        create_dependences_function_fortran_forward();
        _dependences_function_mangled =
            compute_mangled_function_symbol_from_symbol(_dependences_function);
    }

    void TaskProperties::create_dependences_function()
    {
        // Skip this function if the current task doesn't have any task dependence
        if (!_env.any_task_dependence)
            return;

        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
        {
            create_dependences_function_c();
        }
        else if (IS_FORTRAN_LANGUAGE)
        {
            create_dependences_function_fortran();
        }
    }

    Nodecl::NodeclBase TaskProperties::rewrite_expression_using_args(
            TL::Symbol arg,
            Nodecl::NodeclBase expr,
            const TL::ObjectList<TL::Symbol>& local) const
    {
        Nodecl::NodeclBase result = expr.shallow_copy();

        struct RewriteExpression : public Nodecl::ExhaustiveVisitor<void>
        {

            TL::Symbol arg;
            const field_map_t &_field_map;
            const TL::ObjectList<TL::Symbol>& shared;
            const TL::ObjectList<ReductionItem>& reduction;
            const TL::ObjectList<TL::Symbol>& local;
            const TaskProperties& tp;

            RewriteExpression(TL::Symbol arg_,
                    const field_map_t& field_map_,
                    const TL::ObjectList<TL::Symbol> &shared_,
                    const TL::ObjectList<ReductionItem> &reduction_,
                    const TL::ObjectList<TL::Symbol> &local_,
                    const TaskProperties &tp_)
                : arg(arg_), _field_map(field_map_), shared(shared_),
                  reduction(reduction_), local(local_), tp(tp_)
            {
            }

            virtual void visit(const Nodecl::Symbol& node)
            {
                TL::Symbol sym = node.get_symbol();

                // Ignoring local symbols
                if (local.contains(sym))
                    return;

                // Ignoring symbols that are not variables
                if (!sym.is_variable())
                    return;

                field_map_t::const_iterator it = _field_map.find(sym);
                ERROR_CONDITION(it == _field_map.end(),
                        "Symbol '%s' not found in the field map!",
                        sym.get_name().c_str());

                TL::Symbol field(it->second);

                Nodecl::NodeclBase new_expr =
                    Nodecl::ClassMemberAccess::make(
                            arg.make_nodecl(/* set_ref_type */ true, node.get_locus()),
                            field.make_nodecl(node.get_locus()),
                            /* form */ Nodecl::NodeclBase::null(),
                            field.get_type(),
                            node.get_locus());

                if ((shared.contains(sym)
                            || reduction.contains<TL::Symbol>(&ReductionItem::get_symbol, sym)))
                {
                    if (sym.get_type().depends_on_nonconstant_values())
                    {
                        TL::Type updated_cast_type = tp.rewrite_type_using_args(
                                arg,
                                sym.get_type().no_ref().get_pointer_to(),
                                local);

                        new_expr = Nodecl::Conversion::make(
                                new_expr, updated_cast_type, node.get_locus());

                        new_expr.set_text("C");
                    }
                    new_expr = Nodecl::Dereference::make(
                            new_expr,
                            sym.get_type().no_ref().get_lvalue_reference_to(),
                            new_expr.get_locus());
                }

                node.replace(new_expr);
            }

            virtual void visit(const Nodecl::ClassMemberAccess &node)
            {
                walk(node.get_lhs());
            }
        };

        RewriteExpression r(arg, _field_map, _env.shared, _env.reduction, local, *this);
        r.walk(result);

        struct RemoveRedundantRefDerref : public Nodecl::ExhaustiveVisitor<void>
        {
            virtual void visit(const Nodecl::Reference& node)
            {
                if (node.get_rhs().is<Nodecl::Dereference>())
                {
                    node.replace(node.get_rhs().as<Nodecl::Dereference>().get_rhs());
                }
            }

            virtual void visit(const Nodecl::Dereference& node)
            {
                if (node.get_rhs().is<Nodecl::Reference>())
                {
                    node.replace(node.get_rhs().as<Nodecl::Reference>().get_rhs());
                }
            }
        };

        RemoveRedundantRefDerref c;
        c.walk(result);

        return result;
    }

    TL::Type TaskProperties::rewrite_type_using_args(
            TL::Symbol arg,
            TL::Type t,
            const TL::ObjectList<TL::Symbol> &local) const
    {
        if (t.is_array())
        {
            TL::Type elem_type = rewrite_type_using_args(arg, t.array_element(), local);
            if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
            {
                Nodecl::NodeclBase new_size = t.array_get_size();
                new_size = rewrite_expression_using_args(arg, new_size, local);

                return elem_type.get_array_to(new_size, TL::Scope::get_global_scope());
            }
            else if (IS_FORTRAN_LANGUAGE)
            {
                Nodecl::NodeclBase new_lower, new_upper;
                t.array_get_bounds(new_lower, new_upper);

                if (is_saved_expression(new_lower))
                {
                    new_lower = rewrite_expression_using_args(arg, new_lower, local);
                }
                if (is_saved_expression(new_upper))
                {
                    new_upper = rewrite_expression_using_args(arg, new_upper, local);
                }

                return elem_type.get_array_to(new_lower, new_upper, TL::Scope::get_global_scope());
            }
            else
            {
                internal_error("Code unreachable", 0);
            }

        }
        else if (t.is_lvalue_reference())
        {
            return rewrite_type_using_args(arg, t.no_ref(), local).get_lvalue_reference_to();
        }
        else if (t.is_rvalue_reference())
        {
            return rewrite_type_using_args(arg, t.no_ref(), local).get_rvalue_reference_to();
        }
        else if (t.is_pointer())
        {
            return rewrite_type_using_args(arg, t.points_to(), local)
                .get_pointer_to()
                .get_as_qualified_as(t);
        }
        else
        {
            return t;
        }
    }

    void TaskProperties::create_cost_function()
    {
        if (_env.cost_clause.is_null())
            return;

        TL::ObjectList<std::string> parameter_names(1);
        parameter_names[0] = "arg";
        TL::ObjectList<TL::Type> parameter_types(1);
        parameter_types[0] = _info_structure.get_lvalue_reference_to();

        std::string cost_name = get_new_name("nanos6_cost");

        _cost_function
            = SymbolUtils::new_function_symbol(_related_function,
                                               cost_name,
                                               TL::Type::get_size_t_type(),
                                               parameter_names,
                                               parameter_types);

        if (IS_FORTRAN_LANGUAGE)
        {
            _cost_function_mangled =
                compute_mangled_function_symbol_from_symbol(_cost_function);
        }

        Nodecl::NodeclBase cost_function_code, cost_empty_stmt;
        SymbolUtils::build_empty_body_for_function(
            _cost_function, cost_function_code, cost_empty_stmt);

        TL::Scope scope_inside_cost = cost_empty_stmt.retrieve_context();
        TL::Symbol arg = scope_inside_cost.get_symbol_from_name("arg");
        ERROR_CONDITION(!arg.is_valid(), "Invalid symbol", 0);

        Nodecl::NodeclBase computed_cost
            = rewrite_expression_using_args(arg, _env.cost_clause, /* locals */
                    TL::ObjectList<TL::Symbol>());

        if (!computed_cost.get_type().is_same_type(TL::Type::get_size_t_type()))
        {
            computed_cost
                = Nodecl::Conversion::make(computed_cost,
                                           TL::Type::get_size_t_type(),
                                           computed_cost.get_locus());
        }

        Nodecl::NodeclBase return_stmt = Nodecl::ReturnStatement::make(
            computed_cost, computed_cost.get_locus());

        cost_empty_stmt.replace(return_stmt);

        Nodecl::Utils::prepend_to_enclosing_top_level_location(
            _task_body, cost_function_code);
    }

    void TaskProperties::create_priority_function()
    {
        // Skip this function if the current task doesn't have a priority clause
        if (_env.priority_clause.is_null())
            return;

        TL::ObjectList<std::string> parameter_names(1, "arg");
        TL::ObjectList<TL::Type> parameter_types(
                1, _info_structure.get_lvalue_reference_to());

        std::string priority_name = get_new_name("nanos6_priority");

        _priority_function = SymbolUtils::new_function_symbol(
                _related_function,
                priority_name,
                TL::Type::get_size_t_type(),
                parameter_names,
                parameter_types);

        if (IS_FORTRAN_LANGUAGE)
        {
            _priority_function_mangled =
                compute_mangled_function_symbol_from_symbol(_priority_function);
        }

        Nodecl::NodeclBase priority_function_code;
        Nodecl::NodeclBase priority_empty_stmt;

        SymbolUtils::build_empty_body_for_function(
                _priority_function,
                priority_function_code,
                priority_empty_stmt);

        TL::Scope scope_inside_priority =
            priority_empty_stmt.retrieve_context();

        TL::Symbol arg = scope_inside_priority.get_symbol_from_name("arg");
        ERROR_CONDITION(!arg.is_valid(), "Invalid symbol", 0);

        Nodecl::NodeclBase priority_expr = rewrite_expression_using_args(
                arg,
                _env.priority_clause,
                /* local_symbols */ TL::ObjectList<TL::Symbol>());

        if (!priority_expr.get_type()
                .is_same_type(TL::Type::get_size_t_type()))
        {
            priority_expr = Nodecl::Conversion::make(
                    priority_expr,
                    TL::Type::get_size_t_type(),
                    priority_expr.get_locus());
        }

        Nodecl::NodeclBase return_stmt = Nodecl::ReturnStatement::make(
                priority_expr,
                priority_expr.get_locus());

        priority_empty_stmt.replace(return_stmt);

        Nodecl::Utils::prepend_to_enclosing_top_level_location(
                _task_body,
                priority_function_code);
    }

    void TaskProperties::capture_environment(
            TL::Symbol args,
            TL::Scope task_enclosing_scope,
            /* out */
            Nodecl::NodeclBase& captured_env)
    {
        Nodecl::List captured_list;

        Nodecl::NodeclBase vla_offset;
        // 1. Traversing captured variables (firstprivate + other captures)
        for (TL::ObjectList<TL::Symbol>::iterator it = _env.captured_value.begin();
                it != _env.captured_value.end();
                it++)
        {
            ERROR_CONDITION(_field_map.find(*it) == _field_map.end(), "Symbol is not mapped", 0);

            TL::Type lhs_type =
                _field_map[*it].get_type().no_ref().get_lvalue_reference_to();

            Nodecl::NodeclBase lhs =
                Nodecl::ClassMemberAccess::make(
                        Nodecl::Dereference::make(
                            args.make_nodecl(/* set_ref_type */ true),
                            args.get_type().points_to().get_lvalue_reference_to()),
                        _field_map[*it].make_nodecl(),
                        /* member_literal */ Nodecl::NodeclBase::null(),
                        lhs_type);

            Nodecl::List current_captured_stmts;
            if (it->get_type().is_dependent()
                    || (it->get_type().no_ref().is_class() && !it->get_type().no_ref().is_pod()))
            {
                type_t *t = it->get_type().get_internal_type();

                // new (&args.e)E(e);
                Nodecl::NodeclBase new_expr = Nodecl::CxxDepNew::make(
                        Nodecl::CxxParenthesizedInitializer::make(
                            Nodecl::List::make(it->make_nodecl(/* set_ref_type */ true)),
                            get_sequence_of_types(1, &t)),
                        Nodecl::Type::make(it->get_type().no_ref().get_unqualified_type()),
                        Nodecl::List::make(Nodecl::Reference::make(lhs, lhs.get_type().no_ref().get_pointer_to())),
                        it->get_type().no_ref().get_unqualified_type(),
                        /* global */ "");

                current_captured_stmts.append(Nodecl::ExpressionStatement::make(new_expr));
            }
            else if (!it->get_type().no_ref().is_array()
                    && !it->get_type().no_ref().is_function())
            {
                Nodecl::NodeclBase rhs = it->make_nodecl(/* set_ref_type */ true);

                Nodecl::NodeclBase assignment_stmt =
                    Nodecl::ExpressionStatement::make(
                            Nodecl::Assignment::make(
                                lhs,
                                rhs,
                                lhs_type));

                current_captured_stmts.append(assignment_stmt);
            }
            else if (it->get_type().no_ref().is_function())
            {
                Nodecl::NodeclBase rhs = Nodecl::Reference::make(
                        it->make_nodecl(/* set_ref_type */ true),
                        it->get_type().no_ref().get_pointer_to());

                Nodecl::NodeclBase assignment_stmt =
                    Nodecl::ExpressionStatement::make(
                            Nodecl::Assignment::make(
                                lhs,
                                rhs,
                                lhs_type));

                current_captured_stmts.append(assignment_stmt);
            }
            else
            {
                if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
                {
                    if (it->get_type().depends_on_nonconstant_values())
                    {
                        if (vla_offset.is_null())
                        {
                            // Skipping the arguments structure
                            Nodecl::NodeclBase cast = Nodecl::Conversion::make(
                                    Nodecl::Add::make(
                                        args.make_nodecl(/* ser_ref_type */ true),
                                        /* 1, */ const_value_to_nodecl(const_value_get_signed_int(1)),
                                        args.get_type().no_ref()),
                                    TL::Type::get_char_type().get_pointer_to());

                            cast.set_text("C");
                            vla_offset = cast;
                        }

                        // Skipping the extra space allocated for each vla
                        Nodecl::NodeclBase mask_align =
                            const_value_to_nodecl(const_value_get_signed_int(VLA_OVERALLOCATION_ALIGN - 1));

                        // expr = (size_t)(vla_offset + mask_align)
                        Nodecl::NodeclBase cast_expr;
                        cast_expr = Nodecl::Conversion::make(
                                Nodecl::Add::make(
                                    vla_offset,
                                    mask_align,
                                    vla_offset.get_type()),
                                get_size_t_type());
                        cast_expr.set_text("C");

                        // expr = (void *)((size_t)(vla_offset + mask_align) & ~mask_align)
                        cast_expr = Nodecl::Conversion::make(
                                Nodecl::BitwiseAnd::make(
                                    cast_expr,
                                    Nodecl::BitwiseNot::make(
                                        mask_align.shallow_copy(),
                                        mask_align.get_type()),
                                    get_size_t_type()),
                                TL::Type::get_void_type().get_pointer_to());
                        cast_expr.set_text("C");

                        Nodecl::NodeclBase rhs = cast_expr;
                        Nodecl::NodeclBase assignment_stmt = Nodecl::ExpressionStatement::make(
                                Nodecl::Assignment::make(
                                    lhs.shallow_copy(),
                                    rhs,
                                    lhs_type));

                        current_captured_stmts.append(assignment_stmt);

                        // Compute the offset for the next vla symbol (current member + its size)
                        vla_offset = Nodecl::Conversion::make(
                                Nodecl::Add::make(
                                    lhs.shallow_copy(),
                                    Nodecl::Sizeof::make(
                                        Nodecl::Type::make(it->get_type()),
                                        Nodecl::NodeclBase::null(),
                                        get_size_t_type()),
                                    get_size_t_type()),
                                TL::Type::get_char_type().get_pointer_to());
                        vla_offset.set_text("C");
                    }

                    TL::Symbol builtin_memcpy =
                        TL::Scope::get_global_scope().get_symbol_from_name("__builtin_memcpy");

                    ERROR_CONDITION(!builtin_memcpy.is_valid()
                            || !builtin_memcpy.is_function(), "Invalid symbol", 0);

                    Nodecl::NodeclBase ref_lhs =
                        Nodecl::Reference::make(lhs, lhs_type.no_ref().get_pointer_to());

                    Nodecl::NodeclBase rhs = Nodecl::Conversion::make(
                            it->make_nodecl(/* set_ref_type */ true),
                            it->get_type().no_ref().array_element().get_pointer_to());

                    Nodecl::NodeclBase size_of_array;
                    if (it->get_type().depends_on_nonconstant_values())
                    {
                        size_of_array =
                            Nodecl::Sizeof::make(
                                    Nodecl::Type::make(it->get_type()),
                                    Nodecl::NodeclBase::null(),
                                    get_size_t_type());
                    }
                    else
                    {
                        size_of_array =
                            const_value_to_nodecl_with_basic_type(
                                    const_value_get_signed_int(
                                        it->get_type().no_ref().get_size()),
                                    get_size_t_type());
                    }

                    Nodecl::NodeclBase function_call_stmt = Nodecl::ExpressionStatement::make(
                            Nodecl::FunctionCall::make(
                                builtin_memcpy.make_nodecl(/* set_ref_type */ true),
                                Nodecl::List::make(ref_lhs, rhs, size_of_array),
                                /* alternate-name */ Nodecl::NodeclBase::null(),
                                /* function-form */ Nodecl::NodeclBase::null(),
                                TL::Type::get_void_type().get_pointer_to()));

                    current_captured_stmts.append(function_call_stmt);
                }
                else // IS_FORTRAN_LANGUAGE
                {
                    Nodecl::NodeclBase rhs = it->make_nodecl(/* set_ref_type */ true);

                    Nodecl::NodeclBase stmt =
                        Nodecl::ExpressionStatement::make(
                                Nodecl::Assignment::make(
                                    lhs,
                                    rhs,
                                    lhs_type));

                    if (it->is_allocatable())
                    {
                        Nodecl::List allocated_args = Nodecl::List::make(Nodecl::FortranActualArgument::make(rhs.shallow_copy()));
                        TL::Symbol allocated = get_fortran_intrinsic_symbol<1>("allocated", allocated_args, /* is_call */ 0);

                        Nodecl::NodeclBase cond = Nodecl::FunctionCall::make(
                                allocated.make_nodecl(),
                                allocated_args,
                                /* alternate_name */ Nodecl::NodeclBase::null(),
                                /* function_form */ Nodecl::NodeclBase::null(),
                                TL::Type::get_bool_type());

                        stmt = Nodecl::IfElseStatement::make(cond, Nodecl::List::make(stmt), Nodecl::NodeclBase::null());
                    }

                    current_captured_stmts.append(stmt);
                }
            }

            if (IS_FORTRAN_LANGUAGE
                    && it->is_parameter()
                    && it->is_optional())
            {
                Source conditional_capture_src;

                Nodecl::NodeclBase capture_null =
                    Nodecl::ExpressionStatement::make(
                            Nodecl::Assignment::make(
                                lhs.shallow_copy(),
                                Source("MERCURIUM_NULL()").parse_expression(task_enclosing_scope),
                                TL::Type::get_void_type().get_pointer_to()));

                conditional_capture_src
                    << "IF (PRESENT(" << as_symbol(*it) << ")) THEN\n"
                    <<    as_statement(current_captured_stmts)
                    << "ELSE\n"
                    <<    as_statement(capture_null)
                    << "END IF\n"
                    ;

                Nodecl::NodeclBase if_else_stmt =
                    conditional_capture_src.parse_statement(task_enclosing_scope);

               current_captured_stmts = Nodecl::List::make(if_else_stmt);
            }

            captured_list.append(current_captured_stmts);
        }

        // Since we compute the offsets in advance, once all the capture
        // symbols have been treated we can safely free this tree
        if (!vla_offset.is_null())
            nodecl_free(vla_offset.get_internal_nodecl());

        // 2. Traversing SHARED variables
        for (TL::ObjectList<TL::Symbol>::iterator it = _env.shared.begin();
                it != _env.shared.end();
                it++)
        {
            ERROR_CONDITION(_field_map.find(*it) == _field_map.end(),
                    "Symbol is not mapped", 0);

            Nodecl::NodeclBase rhs = it->make_nodecl(/* set_ref_type */ true);

            if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
            {
                rhs = Nodecl::Reference::make(
                        rhs,
                        rhs.get_type().no_ref().get_pointer_to());
            }
            else // IS_FORTRAN_LANGUAGE
            {
                if (it->get_type().no_ref().is_pointer()
                    || it->is_allocatable())
                {
                    TL::Symbol ptr_of_sym = fortran_get_function_ptr_of(
                        *it,
                        _related_function.get_related_scope(),
                        _phase->get_extra_c_code());

                    rhs = Nodecl::FunctionCall::make(
                        ptr_of_sym.make_nodecl(/* set_ref_type */ true),
                        Nodecl::List::make(
                            it->make_nodecl(/* set_ref_type */ true)),
                        /* alternate_name */ Nodecl::NodeclBase::null(),
                        /* function_form */ Nodecl::NodeclBase::null(),
                        ptr_of_sym.get_type().returns());
                }
                else if (it->get_type().no_ref().is_array()
                         && it->get_type().no_ref().array_requires_descriptor())
                {
                    TL::Symbol descriptor_field = _array_descriptor_map[*it];
                    ERROR_CONDITION(!descriptor_field.is_valid(),
                                    "Array descriptor field not found",
                                    0);

                    TL::Symbol copy_function
                        = fortran_get_copy_descriptor_function(
                            /* dest */ descriptor_field,
                            /* source */ *it,
                            _related_function.get_related_scope(),
                            _phase->get_extra_c_code());

                    Nodecl::NodeclBase dest = Nodecl::ClassMemberAccess::make(
                        Nodecl::Dereference::make(
                            args.make_nodecl(/* set_ref_type */ true),
                            args.get_type()
                                .points_to()
                                .get_lvalue_reference_to()),
                        descriptor_field.make_nodecl(),
                        /* member_literal */ Nodecl::NodeclBase::null(),
                        descriptor_field.get_type().get_lvalue_reference_to());

                    Nodecl::NodeclBase capture_descriptor_stmt
                        = Nodecl::ExpressionStatement::make(
                            Nodecl::FunctionCall::make(
                                copy_function.make_nodecl(/* set_ref */ true),
                                Nodecl::List::make(
                                    dest, it->make_nodecl(/* set_ref */ true)),
                                Nodecl::NodeclBase::null(),
                                Nodecl::NodeclBase::null(),
                                get_void_type()));

                    captured_list.append(capture_descriptor_stmt);

                    rhs = Nodecl::Reference::make(
                        dest.shallow_copy(),
                        descriptor_field.get_type().get_pointer_to());
                }
                else
                {
                    rhs = Nodecl::Reference::make(
                        rhs, rhs.get_type().no_ref().get_pointer_to());
                }
            }

            TL::Type lhs_type =
                _field_map[*it].get_type().no_ref().get_lvalue_reference_to();

            Nodecl::NodeclBase lhs =
                Nodecl::ClassMemberAccess::make(
                        Nodecl::Dereference::make(
                            args.make_nodecl(/* set_ref_type */ true),
                            args.get_type().points_to().get_lvalue_reference_to()),
                        _field_map[*it].make_nodecl(),
                        /* member_literal */ Nodecl::NodeclBase::null(),
                        lhs_type);

            Nodecl::NodeclBase current_captured_stmt = Nodecl::ExpressionStatement::make(
                    Nodecl::Assignment::make(
                        lhs,
                        rhs,
                        lhs_type));

            if (IS_FORTRAN_LANGUAGE
                    && it->is_parameter()
                    && it->is_optional())
            {
                Nodecl::NodeclBase capture_null = 
                    Nodecl::ExpressionStatement::make(
                            Nodecl::Assignment::make(
                                lhs.shallow_copy(),
                                Source("MERCURIUM_NULL()").parse_expression(task_enclosing_scope),
                                TL::Type::get_void_type().get_pointer_to()));

                Source conditional_capture_src;

                conditional_capture_src
                    << "IF (PRESENT(" << as_symbol(*it) << ")) THEN\n"
                    <<    as_statement(current_captured_stmt)
                    << "ELSE\n"
                    <<    as_statement(capture_null)
                    << "END IF\n"
                    ;

                current_captured_stmt =
                    conditional_capture_src.parse_statement(task_enclosing_scope);
            }

            captured_list.append(current_captured_stmt);
        }

        // 3. Traversing REDUCTION variables
        for (TL::ObjectList<ReductionItem>::iterator it = _env.reduction.begin();
                it != _env.reduction.end();
                it++)
        {
            ERROR_CONDITION(_field_map.find(it->symbol) == _field_map.end(),
                    "Symbol is not mapped", 0);

            Nodecl::NodeclBase rhs = it->symbol.make_nodecl(/* set_ref_type */ true);
            rhs = Nodecl::Reference::make(
                    rhs,
                    rhs.get_type().no_ref().get_pointer_to());

            std::string update_address_fun_name = "nanos_get_original_reduction_address";

            TL::Symbol update_address_fun =
                TL::Scope::get_global_scope().get_symbol_from_name(update_address_fun_name);
            if (!update_address_fun.is_valid())
            {
                fatal_error(
                        "'%s' function not found while trying to register dependences\n",
                        update_address_fun_name.c_str());
            }

            rhs = Nodecl::Conversion::make(
                    Nodecl::FunctionCall::make(
                        update_address_fun.make_nodecl(/* set_ref_type */ true),
                        Nodecl::List::make(rhs),
                        /* alternate_name */ Nodecl::NodeclBase::null(),
                        /* function_form */ Nodecl::NodeclBase::null(),
                        TL::Type::get_void_type().get_pointer_to()),
                    rhs.get_type());

            rhs.set_text("C");

            TL::Type lhs_type =
                _field_map[it->symbol].get_type().no_ref().get_lvalue_reference_to();

            Nodecl::NodeclBase lhs =
                Nodecl::ClassMemberAccess::make(
                        Nodecl::Dereference::make(
                            args.make_nodecl(/* set_ref_type */ true),
                            args.get_type().points_to().get_lvalue_reference_to()),
                        _field_map[it->symbol].make_nodecl(),
                        /* member_literal */ Nodecl::NodeclBase::null(),
                        lhs_type);

            Nodecl::NodeclBase current_captured_stmt = Nodecl::ExpressionStatement::make(
                    Nodecl::Assignment::make(
                        lhs,
                        rhs,
                        lhs_type));

            captured_list.append(current_captured_stmt);
        }

        if (IS_FORTRAN_LANGUAGE)
        {

            for (TL::ObjectList<TL::Symbol>::iterator it = _env.private_.begin();
                    it != _env.private_.end();
                    it++)
            {
                if (!symbol_entity_specs_get_is_allocatable(it->get_internal_symbol()))
                        continue;

                Nodecl::NodeclBase allocate_stmt;
                {
                    std::stringstream shape_list;
                    if (it->get_type().no_ref().is_array())
                    {
                        TL::Type array_type = it->get_type().no_ref();
                        int dim = 1;
                        shape_list << "(";
                        while (array_type.is_array())
                        {
                            shape_list
                                << (dim > 1 ? ", " : "")
                                << "LBOUND(" << it->get_name() << ", " << dim << ")"
                                << " : "
                                << "UBOUND(" << it->get_name() << ", " << dim << ")"
                                ;

                            array_type = array_type.array_element();
                            dim++;
                        }
                        shape_list << ")";
                    }

                    TL::Source allocate_src;
                    allocate_src << "ALLOCATE(" << as_symbol(args) << " % " << it->get_name() << shape_list.str() << ")\n";
                    allocate_stmt = allocate_src.parse_statement(task_enclosing_scope);
                }

                Nodecl::List actual_arguments = Nodecl::List::make(Nodecl::FortranActualArgument::make(it->make_nodecl(true)));
                TL::Symbol allocated = get_fortran_intrinsic_symbol<1>("allocated", actual_arguments, /* is_call */ 0);

                Nodecl::NodeclBase cond = Nodecl::FunctionCall::make(
                        allocated.make_nodecl(),
                        actual_arguments,
                        /* alternate_name */ Nodecl::NodeclBase::null(),
                        /* function_form */ Nodecl::NodeclBase::null(),
                        TL::Type::get_bool_type());

                Nodecl::NodeclBase if_stmt =
                    Nodecl::IfElseStatement::make(cond, allocate_stmt, Nodecl::NodeclBase::null());

                captured_list.append(if_stmt);
            }
        }

        captured_env = captured_list;
    }

    namespace
    {
        // Helper function that computes 'taskloop_bounds_ptr->field_name = value;'
        Nodecl::NodeclBase capture_taskloop_information_field(
                const TL::Symbol& taskloop_bounds_ptr,
                const GetField& get_field,
                const std::string& field_name,
                Nodecl::NodeclBase value)
        {
            Nodecl::NodeclBase field = get_field(field_name);
            return Nodecl::ExpressionStatement::make(
                    Nodecl::Assignment::make(
                        Nodecl::ClassMemberAccess::make(
                            Nodecl::Dereference::make(
                                taskloop_bounds_ptr.make_nodecl(/*set_ref_type*/ true),
                                taskloop_bounds_ptr.get_type().points_to().get_lvalue_reference_to()),
                            field,
                            /*member literal*/ Nodecl::NodeclBase::null(),
                            field.get_type()),
                        value,
                        value.get_type()));
        }
    }

    void TaskProperties::capture_taskloop_information(
            TL::Symbol taskloop_bounds_ptr,
            /* out */
            Nodecl::NodeclBase& stmts) const
    {
        ERROR_CONDITION(!_env.is_taskloop, "The construct should be a Taskloop\n", 0);

        TL::Type class_type = taskloop_bounds_ptr.get_type().points_to();
        ERROR_CONDITION(!class_type.is_class(), "Unexpected type\n", 0);

        TL::ObjectList<TL::Symbol> nonstatic_data_members = class_type.get_nonstatic_data_members();
        GetField get_field(nonstatic_data_members);

        Nodecl::NodeclBase original_lower_bound, original_upper_bound, original_step, chunksize;
        original_lower_bound = _taskloop_info.lower_bound.shallow_copy();
        original_upper_bound = _taskloop_info.upper_bound.shallow_copy();
        original_step = _taskloop_info.step.shallow_copy();
        chunksize = _taskloop_info.chunksize.shallow_copy();

        Nodecl::List new_stmts;

        new_stmts.append(
                capture_taskloop_information_field(
                    taskloop_bounds_ptr, get_field, "lower_bound", original_lower_bound));

        new_stmts.append(
                capture_taskloop_information_field(
                    taskloop_bounds_ptr, get_field, "upper_bound", original_upper_bound));

        new_stmts.append(
                capture_taskloop_information_field(
                    taskloop_bounds_ptr, get_field, "step", original_step));

        new_stmts.append(
                capture_taskloop_information_field(
                    taskloop_bounds_ptr, get_field, "chunksize", chunksize));


        stmts = new_stmts;
    }

    void TaskProperties::fortran_add_types(TL::Scope dest_scope)
    {
        TL::ObjectList<TL::Symbol> all_syms;
        all_syms.append(_env.shared);
        all_syms.append(_env.captured_value);

        TL::Nanos6::fortran_add_types(all_syms, dest_scope);
    }

    void TaskProperties::compute_release_statements(/* out */ Nodecl::List& release_stmts)
    {
        struct ReleaseSet
        {
            TL::ObjectList<Nodecl::NodeclBase> &dep_list;
            std::string func_name;
        } deps[] = {
            { _env.dep_in,    "nanos_release_read_"              },
            { _env.dep_out,   "nanos_release_write_"             },
            { _env.dep_inout, "nanos_release_readwrite_"         },

            { _env.dep_weakin,    "nanos_release_weak_read_"      },
            { _env.dep_weakout,   "nanos_release_weak_write_"     },
            { _env.dep_weakinout, "nanos_release_weak_readwrite_" },

            { _env.dep_commutative, "nanos_release_commutative_" },
            { _env.dep_concurrent,  "nanos_release_concurrent_"  },

           // { dep_reduction, "nanos_release_reduction_" },
        };

        TL::Scope global_context = TL::Scope::get_global_scope();

        for (ReleaseSet *release_set = deps;
                release_set != (ReleaseSet*)(&deps + 1);
                release_set++)
        {
            TL::ObjectList<Nodecl::NodeclBase> &dep_list = release_set->dep_list;
            if (dep_list.empty())
                continue;

            for (TL::ObjectList<Nodecl::NodeclBase>::iterator
                    it = dep_list.begin();
                    it != dep_list.end();
                    it++)
            {
                TL::DataReference data_ref = *it;
                TL::Type data_type = data_ref.get_data_type();

                TL::Symbol release_fun;
                {
                    int max_dimensions = _phase->nanos6_api_max_dimensions();
                    ERROR_CONDITION(data_type.is_array() &&
                            (data_type.get_num_dimensions() > max_dimensions),
                            "Maximum number of data dimensions allowed is %d",
                            max_dimensions);

                    int num_dims_dep = data_type.is_array() ? data_type.get_num_dimensions() : 1;
                    std::stringstream ss;
                    ss << release_set->func_name << num_dims_dep;

                    release_fun = global_context.get_symbol_from_name(ss.str());
                    if (!release_fun.is_valid())
                    {
                        fatal_error(
                                "'%s' function not found while trying to release dependences\n",
                                ss.str().c_str());
                    }
                }

                ERROR_CONDITION(data_ref.is_multireference(), "Unexpected multi-dependence in a release construct\n", 0);

                TL::ObjectList<Nodecl::NodeclBase> arguments;
                compute_base_address_and_dimensionality_information(data_ref, arguments);
                Nodecl::NodeclBase call_to_release = Nodecl::ExpressionStatement::make(
                        Nodecl::FunctionCall::make(
                            release_fun.make_nodecl(/* set_ref_type */ true),
                            Nodecl::List::make(arguments),
                            /* alternate_name */ Nodecl::NodeclBase::null(),
                            /* function_form */ Nodecl::NodeclBase::null(),
                            get_void_type()));

                release_stmts.append(call_to_release);
            }
        }
    }

    bool TaskProperties::is_saved_expression(Nodecl::NodeclBase n)
    {
        return (n.no_conv().is<Nodecl::Symbol>()
                && n.no_conv().get_symbol().is_saved_expression());
    }

    bool TaskProperties::is_taskloop() const
    {
        return _env.is_taskloop;
    }
} }
