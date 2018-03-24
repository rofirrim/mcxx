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


#include "tl-nanos6-lower.hpp"
#include "tl-nanos6-task-properties.hpp"
#include "tl-nanos6-fortran-support.hpp"
#include "tl-nanos6-interface.hpp"

#include "tl-counters.hpp"
#include "tl-source.hpp"

#include "cxx-exprtype.h"

namespace TL { namespace Nanos6 {

    void Lower::visit(const Nodecl::OpenMP::Task& node)
    {
        walk(node.get_statements());
        Nodecl::NodeclBase serial_stmts;

        // If disabled, act normally
        if (!_phase->_final_clause_transformation_disabled)
        {
            std::map<Nodecl::NodeclBase, Nodecl::NodeclBase>::iterator it = _final_stmts_map.find(node);
            ERROR_CONDITION(it == _final_stmts_map.end(), "Invalid serial statements", 0);
            serial_stmts = it->second;
        }

        lower_task(node, serial_stmts);
    }

    // Substitute the task node for an ifelse for when using final
    void Lower::lower_task(const Nodecl::OpenMP::Task& node, Nodecl::NodeclBase& serial_stmts)
    {
        ERROR_CONDITION(serial_stmts.is_null()
                && !_phase->_final_clause_transformation_disabled,
                "Invalid serial statement for a task", 0);

        Nodecl::OpenMP::Task new_task = node;
        if (!_phase->_final_clause_transformation_disabled)
        {

            Nodecl::NodeclBase stmts = node.get_statements();

            // Wrap the function call into if (nanos_in_final())
            TL::Symbol nanos_in_final_sym =
                TL::Scope::get_global_scope().get_symbol_from_name("nanos_in_final");
            ERROR_CONDITION(!nanos_in_final_sym .is_valid()
                    || !nanos_in_final_sym.is_function(),
                    "Invalid symbol", 0);

            Nodecl::NodeclBase call_to_nanos_in_final = Nodecl::FunctionCall::make(
                /* called */ nanos_in_final_sym.make_nodecl(/* set_ref_type */ true, node.get_locus()),
                /* arguments */ Nodecl::NodeclBase::null(),
                /* alternate_name */Nodecl::NodeclBase::null(),
                /* function_form */ Nodecl::NodeclBase::null(),
                TL::Type::get_int_type());

            new_task = Nodecl::OpenMP::Task::make(node.get_environment(), stmts, node.get_locus());

            Scope sc = node.retrieve_context();
            Scope not_final_context = new_block_context(sc.get_decl_context());

            Nodecl::NodeclBase not_final_compound_stmt = Nodecl::Context::make(
                Nodecl::List::make(
                    Nodecl::CompoundStatement::make(
                        Nodecl::List::make(new_task),
                        /* finally */ Nodecl::NodeclBase::null(),
                        node.get_locus())),
                not_final_context,
                node.get_locus());

            Scope in_final_context = new_block_context(sc.get_decl_context());
            Nodecl::NodeclBase in_final_compound_stmts = Nodecl::Context::make(
                Nodecl::List::make(
                    Nodecl::CompoundStatement::make(
                        serial_stmts,
                        /* finally */ Nodecl::NodeclBase::null(),
                        node.get_locus())),
                in_final_context,
                node.get_locus());

            Nodecl::NodeclBase if_in_final = Nodecl::IfElseStatement::make(
                    Nodecl::Different::make(
                        call_to_nanos_in_final,
                        const_value_to_nodecl_with_basic_type(
                            const_value_get_signed_int(0),
                            get_size_t_type()),
                        get_bool_type()),
                    Nodecl::List::make(in_final_compound_stmts),
                    Nodecl::List::make(not_final_compound_stmt)
                );

            node.replace(if_in_final);

            // Traverse the serial statements since they may contain additional pragmas
            walk(serial_stmts);
        }

        lower_task(new_task);
    }

    // Creates the task instantiation and submission
    void Lower::lower_task(const Nodecl::OpenMP::Task& node)
    {
        TaskProperties task_properties(node, _phase, this);

        Nodecl::NodeclBase args_size;
        TL::Type data_env_struct;
        bool requires_initialization;
        task_properties.create_environment_structure(
                /* out */
                data_env_struct,
                args_size,
                requires_initialization);

        TL::Symbol task_invocation_info;
        task_properties.create_task_invocation_info(
                /* out */
                task_invocation_info);

        TL::Symbol implementations;
        task_properties.create_task_implementations_info(
                /* out */
                implementations);

        TL::Symbol task_info;
        task_properties.create_task_info(
                implementations,
                /* out */
                task_info);

        TL::Scope sc = node.retrieve_context();

        TL::Symbol args;
        {
            TL::Counter &counter = TL::CounterManager::get_counter("nanos6-task-args");
            std::stringstream ss;
            ss << "nanos_data_env_" << (int)counter;
            counter++;

            args = sc.new_symbol(ss.str());
            args.get_internal_symbol()->kind = SK_VARIABLE;
            args.set_type(data_env_struct.get_pointer_to());
            symbol_entity_specs_set_is_user_declared(
                    args.get_internal_symbol(), 1);
        }

        TL::Symbol task_ptr;
        {
            TL::Counter &counter = TL::CounterManager::get_counter("nanos6-task-ptr");
            std::stringstream ss;
            ss << "nanos_task_ptr_" << (int)counter;
            counter++;

            task_ptr = sc.new_symbol(ss.str());
            task_ptr.get_internal_symbol()->kind = SK_VARIABLE;
            task_ptr.set_type(TL::Type::get_void_type().get_pointer_to());
            symbol_entity_specs_set_is_user_declared(
                    task_ptr.get_internal_symbol(), 1);
        }

        Nodecl::List new_stmts;

        // Create task
        {
            if (IS_CXX_LANGUAGE)
            {
                new_stmts.append(Nodecl::CxxDef::make(Nodecl::NodeclBase::null(), args));
                new_stmts.append(Nodecl::CxxDef::make(Nodecl::NodeclBase::null(), task_ptr));
            }

            TL::Symbol nanos_create_task_sym =
                TL::Scope::get_global_scope().get_symbol_from_name("nanos_create_task");
            ERROR_CONDITION(!nanos_create_task_sym.is_valid()
                    || !nanos_create_task_sym.is_function(),
                    "Invalid symbol", 0);

            // void nanos_create_task(
            //         nanos_task_info *task_info,
            //         nanos_task_invocation_info *task_invocation_info,
            //         size_t args_block_size,
            //         /* OUT */ void **args_block_pointer,
            //         /* OUT */ void **task_pointer,
            //         size_t flags);

            Nodecl::List create_task_args;

            // &task_info
            Nodecl::NodeclBase task_info_ptr =
                Nodecl::Reference::make(
                    task_info.make_nodecl(
                        /* set_ref_type */ true,
                        node.get_locus()),
                    task_info.get_type().get_pointer_to(),
                    node.get_locus());

            create_task_args.append(task_info_ptr);


            // &task_invocation_info
            Nodecl::NodeclBase task_invocation_info_ptr =
                Nodecl::Reference::make(
                        task_invocation_info.make_nodecl(
                            /* set_ref_type */ true,
                            node.get_locus()),
                        task_invocation_info.get_type().get_pointer_to(),
                        node.get_locus());

            create_task_args.append(task_invocation_info_ptr);


            //args_size
            create_task_args.append(args_size);


            // (void**)&args
            Nodecl::NodeclBase cast;
            Nodecl::NodeclBase args_ptr_out =
                cast = Nodecl::Conversion::make(
                        Nodecl::Reference::make(
                            args.make_nodecl(
                                /* set_ref_type */ true,
                                node.get_locus()),
                            args.get_type().get_pointer_to(),
                            node.get_locus()),
                        TL::Type::get_void_type().get_pointer_to().get_pointer_to(),
                        node.get_locus());

            cast.set_text("C");
            create_task_args.append(args_ptr_out);


            // &task_ptr
            Nodecl::NodeclBase task_ptr_out =
                Nodecl::Reference::make(
                        task_ptr.make_nodecl(
                            /* set_ref_type */ true,
                            node.get_locus()),
                        task_ptr.get_type().get_pointer_to(),
                        node.get_locus());

            create_task_args.append(task_ptr_out);


            // task_flags
            Nodecl::NodeclBase flags_nodecl;
            {
                TL::Counter &counter = TL::CounterManager::get_counter("nanos6-task-flags");
                std::stringstream ss;
                ss << "task_flags_" << (int)counter;
                counter++;

                TL::Symbol task_flags = sc.new_symbol(ss.str());
                task_flags.get_internal_symbol()->kind = SK_VARIABLE;
                task_flags.get_internal_symbol()->type_information = TL::Type::get_size_t_type().get_internal_type();
                symbol_entity_specs_set_is_user_declared(task_flags.get_internal_symbol(), 1);

                if (IS_CXX_LANGUAGE)
                    new_stmts.append(Nodecl::CxxDef::make(Nodecl::NodeclBase::null(), task_flags));

                Nodecl::NodeclBase task_flags_stmts;
                task_properties.compute_task_flags(task_flags, task_flags_stmts);
                new_stmts.append(task_flags_stmts);

                flags_nodecl = task_flags.make_nodecl(/*set_ref_type */ true);
                create_task_args.append(flags_nodecl);
            }

            Nodecl::NodeclBase call_to_nanos_create_task =
                Nodecl::ExpressionStatement::make(
                        Nodecl::FunctionCall::make(
                            nanos_create_task_sym.make_nodecl(/* set_ref_type */ true, node.get_locus()),
                            create_task_args,
                            /* alternate name */ Nodecl::NodeclBase::null(),
                            /* function form  */ Nodecl::NodeclBase::null(),
                            TL::Type::get_void_type(),
                            node.get_locus()),
                        node.get_locus());

            new_stmts.append(call_to_nanos_create_task);
        }

        if (requires_initialization)
        {
            // FORTRAN ONLY
            ERROR_CONDITION(IS_CXX_LANGUAGE || IS_C_LANGUAGE, "Unreachable code\n", 0);

            TL::Symbol nanos6_bzero_sym =
                TL::Scope::get_global_scope().get_symbol_from_name("nanos6_bzero");
            ERROR_CONDITION(!nanos6_bzero_sym.is_valid()
                    || !nanos6_bzero_sym.is_function(),
                    "Invalid symbol", 0);

            //  TYPE(ARGS_T), POINTER :: ARGS
            //
            //  What we want to set to zero is the storage of this pointer, not
            //  the descriptor itself: LOC(ARGS)
            Nodecl::NodeclBase address_of_args =
                Nodecl::Reference::make(
                        Nodecl::Dereference::make(
                            args.make_nodecl(/*set_ref_type*/true),
                            args.get_type().points_to()),
                        args.get_type().no_ref());

            Nodecl::NodeclBase call_to_nanos6_bzero =
                Nodecl::ExpressionStatement::make(
                        Nodecl::FunctionCall::make(
                           nanos6_bzero_sym.make_nodecl( /* set_ref_type */ true),
                           Nodecl::List::make(
                               address_of_args,
                               args_size.shallow_copy()),
                           /* alternate symbol */ Nodecl::NodeclBase::null(),
                           /* alternate symbol */ Nodecl::NodeclBase::null(),
                           TL::Type::get_void_type(),
                           node.get_locus()),
                        node.get_locus());

            new_stmts.append(call_to_nanos6_bzero);
        }

        // Capture environment
        {
            Nodecl::NodeclBase capture_env;
            task_properties.capture_environment(
                    args,
                    sc,
                    /* out */ capture_env);

            new_stmts.append(capture_env);
        }

        if (task_properties.task_is_loop())
        {
            Interface::family_must_be_at_least("nanos6_taskloop_api", 1, "the 'loop' construct");

            TL::Symbol nanos_register_loop_sym =
                TL::Scope::get_global_scope().get_symbol_from_name("nanos_register_taskloop_bounds");
            ERROR_CONDITION(!nanos_register_loop_sym.is_valid()
                    || !nanos_register_loop_sym.is_function(), "Invalid symbol", 0);

            ERROR_CONDITION(!node.get_statements().is<Nodecl::List>(), "Unexpected node\n", 0);
            Nodecl::NodeclBase stmt = node.get_statements().as<Nodecl::List>().front();
            ERROR_CONDITION(!stmt.is<Nodecl::Context>(), "Unexpected node\n", 0);
            stmt = stmt.as<Nodecl::Context>().get_in_context().as<Nodecl::List>().front();
            ERROR_CONDITION(!stmt.is<Nodecl::ForStatement>(), "Unexpected node\n", 0);

            TL::ObjectList<TL::Symbol> params = nanos_register_loop_sym.get_related_symbols();
            Nodecl::List reg_loop_args;

            reg_loop_args.append(task_ptr.make_nodecl(/*ref_type*/true));

            Nodecl::NodeclBase lower_bound = task_properties.get_lower_bound().shallow_copy();
            if (IS_FORTRAN_LANGUAGE)
                lower_bound = Nodecl::Conversion::make(lower_bound, params[1].get_type());
            reg_loop_args.append(lower_bound);

            Nodecl::NodeclBase upper_bound = task_properties.get_upper_bound().shallow_copy();
            if (IS_FORTRAN_LANGUAGE)
                upper_bound = Nodecl::Conversion::make(upper_bound, params[2].get_type());
            reg_loop_args.append(upper_bound);

            Nodecl::NodeclBase step = task_properties.get_step().shallow_copy();
            if (IS_FORTRAN_LANGUAGE)
                step = Nodecl::Conversion::make(step, params[3].get_type());
            reg_loop_args.append(step);

            Nodecl::NodeclBase chunksize = task_properties.get_chunksize().shallow_copy();
            if (IS_FORTRAN_LANGUAGE)
                chunksize = Nodecl::Conversion::make(chunksize, params[4].get_type());
            reg_loop_args.append(chunksize);

            new_stmts.append(
                Nodecl::ExpressionStatement::make(
                    Nodecl::FunctionCall::make(
                        nanos_register_loop_sym.make_nodecl(/*ref_type*/true),
                        reg_loop_args,
                        /* alternate symbol */ Nodecl::NodeclBase::null(),
                        /* function form */ Nodecl::NodeclBase::null(),
                        TL::Type::get_void_type(),
                        node.get_locus()),
                    node.get_locus()));
        }

        // Submit the created task
        {
            TL::Symbol nanos_submit_task_sym =
                TL::Scope::get_global_scope().get_symbol_from_name("nanos_submit_task");
            ERROR_CONDITION(!nanos_submit_task_sym.is_valid()
                    || !nanos_submit_task_sym.is_function(),
                    "Invalid symbol", 0);

            Nodecl::NodeclBase task_ptr_arg = task_ptr.make_nodecl(/* set_ref_type */ true);
            task_ptr_arg = ::cxx_nodecl_make_conversion(
                    task_ptr_arg.get_internal_nodecl(),
                    task_ptr.get_type().no_ref().get_internal_type(),
                    TL::Scope::get_global_scope().get_decl_context(),
                    node.get_locus());

            Nodecl::NodeclBase new_task =
                Nodecl::ExpressionStatement::make(
                        Nodecl::FunctionCall::make(
                            nanos_submit_task_sym.make_nodecl(/* set_ref_type */ true),
                            Nodecl::List::make(task_ptr_arg),
                            /* alternate symbol */ Nodecl::NodeclBase::null(),
                            /* function form */ Nodecl::NodeclBase::null(),
                            TL::Type::get_void_type(),
                            node.get_locus()),
                        node.get_locus());

            new_stmts.append(new_task);
        }
        node.replace(new_stmts);
    }

} }
