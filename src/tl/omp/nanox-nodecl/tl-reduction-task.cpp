/*--------------------------------------------------------------------
  (C) Copyright 2006-2013 Barcelona Supercomputing Center
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

#include "tl-nanos.hpp"
#include "tl-nodecl-utils-fortran.hpp"
#include "tl-counters.hpp"
#include "tl-outline-info.hpp"
#include "tl-lowering-visitor.hpp"
#include "tl-nanox-ptr.hpp"

namespace TL { namespace Nanox {

    struct BasicReductionExpandVisitor1 : public Nodecl::ExhaustiveVisitor<void>
    {
        private:
            TL::Symbol _orig_omp_in, _new_omp_in, _orig_omp_out, _new_omp_out;
        public:
        BasicReductionExpandVisitor1(
                TL::Symbol orig_omp_in,
                TL::Symbol new_omp_in,
                TL::Symbol orig_omp_out,
                TL::Symbol new_omp_out)
            : _orig_omp_in(orig_omp_in),
            _new_omp_in(new_omp_in),
            _orig_omp_out(orig_omp_out),
            _new_omp_out(new_omp_out)
        {
        }

        void visit(const Nodecl::Symbol &node)
        {
            bool must_expand = false;
            TL::Symbol sym = node.get_symbol();
            TL::Symbol new_sym;
            if (sym == _orig_omp_in)
            {
                must_expand = true;
                new_sym = _new_omp_in;
            }
            else if (sym == _orig_omp_out)
            {
                must_expand = true;
                new_sym = _new_omp_out;
            }

            if (!must_expand)
                return;

            Nodecl::NodeclBase new_sym_ref = Nodecl::Symbol::make(new_sym);
            new_sym_ref.set_type(new_sym.get_type());

            TL::Type new_type = new_sym.get_type().no_ref();
            if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
            {
                new_type = new_type.points_to().get_lvalue_reference_to();
                node.replace(
                        Nodecl::Dereference::make(
                            new_sym_ref,
                            new_type));

            }
            else if (IS_FORTRAN_LANGUAGE)
            {
                node.replace(new_sym_ref);
            }
            else
            {
                internal_error("Code unreachable", 0);
            }
        }
    };

    static TL::Symbol create_reduction_function_c(
            OpenMP::Reduction* red,
            Nodecl::NodeclBase construct,
            LoweringVisitor::reduction_map_t & reduction_map)
    {

        LoweringVisitor::reduction_map_t::iterator it = reduction_map.find(red);
        if (it != reduction_map.end())
        {
            return it->second;
        }

        std::string fun_name;
        {
            std::stringstream ss;
            ss << "nanos_red_" << red << "_" << simple_hash_str(construct.get_filename().c_str());
            fun_name = ss.str();
        }

        Nodecl::NodeclBase function_body;
        Source src;
        src << "void " << fun_name << "("
            << as_type(red->get_type()) << "* omp_out, "
            << as_type(red->get_type()) << "* omp_in)"
            << "{"
            <<    statement_placeholder(function_body)
            << "}"
            ;

        Nodecl::NodeclBase function_code = src.parse_global(construct.retrieve_context().get_global_scope());

        TL::Scope inside_function = ReferenceScope(function_body).get_scope();
        TL::Symbol param_omp_in = inside_function.get_symbol_from_name("omp_in");
        ERROR_CONDITION(!param_omp_in.is_valid(), "Symbol omp_in not found", 0);
        TL::Symbol param_omp_out = inside_function.get_symbol_from_name("omp_out");
        ERROR_CONDITION(!param_omp_out.is_valid(), "Symbol omp_out not found", 0);

        TL::Symbol function_sym = inside_function.get_symbol_from_name(fun_name);
        ERROR_CONDITION(!function_sym.is_valid(), "Symbol %s not found", fun_name.c_str());

        Nodecl::NodeclBase expanded_combiner = red->get_combiner().shallow_copy();
        BasicReductionExpandVisitor1 expander_visitor(
                red->get_omp_in(),
                param_omp_in,
                red->get_omp_out(),
                param_omp_out);

        expander_visitor.walk(expanded_combiner);

        function_body.replace(
                Nodecl::List::make(Nodecl::ExpressionStatement::make(expanded_combiner)));

       reduction_map[red] = function_sym;

        // As the reduction function is needed during the instantiation of
        // the task, this function should be inserted before the construct
        Nodecl::Utils::prepend_to_enclosing_top_level_location(construct, function_code);

        return function_sym;
    }

    static TL::Symbol create_reduction_function_fortran(
            OpenMP::Reduction* red,
            Nodecl::NodeclBase construct,
            LoweringVisitor::reduction_map_t & reduction_map)
    {
        LoweringVisitor::reduction_map_t::iterator it = reduction_map.find(red);
        if (it != reduction_map.end())
        {
            return it->second;
        }

        std::string fun_name;
        {
            std::stringstream ss;
            ss << "nanos_red_" << red << "_" << simple_hash_str(construct.get_filename().c_str());
            fun_name = ss.str();
        }

        Nodecl::NodeclBase function_body;
        Source src;

        src << "SUBROUTINE " << fun_name << "(omp_out, omp_in)\n"
            <<    "IMPLICIT NONE\n"
            <<    as_type(red->get_type()) << " :: omp_out\n"
            <<    as_type(red->get_type()) << " :: omp_in\n"
            <<    statement_placeholder(function_body) << "\n"
            << "END SUBROUTINE " << fun_name << "\n";
        ;

        Nodecl::NodeclBase function_code = src.parse_global(construct);

        TL::Scope inside_function = ReferenceScope(function_body).get_scope();
        TL::Symbol param_omp_in = inside_function.get_symbol_from_name("omp_in");
        ERROR_CONDITION(!param_omp_in.is_valid(), "Symbol omp_in not found", 0);
        TL::Symbol param_omp_out = inside_function.get_symbol_from_name("omp_out");
        ERROR_CONDITION(!param_omp_out.is_valid(), "Symbol omp_out not found", 0);

        TL::Symbol function_sym = inside_function.get_symbol_from_name(fun_name);
        ERROR_CONDITION(!function_sym.is_valid(), "Symbol %s not found", fun_name.c_str());

        Nodecl::NodeclBase expanded_combiner =
            red->get_combiner().shallow_copy();
        BasicReductionExpandVisitor1 expander_visitor(
                red->get_omp_in(),
                param_omp_in,
                red->get_omp_out(),
                param_omp_out);

        expander_visitor.walk(expanded_combiner);

        function_body.replace(
                Nodecl::List::make(Nodecl::ExpressionStatement::make(expanded_combiner)));

        reduction_map[red] = function_sym;

        if (IS_FORTRAN_LANGUAGE)
        {
            Nodecl::Utils::Fortran::append_used_modules(construct.retrieve_context(),
                    function_sym.get_related_scope());
        }

        // As the reduction function is needed during the instantiation of
        // the task, this function should be inserted before the construct
        Nodecl::Utils::prepend_to_enclosing_top_level_location(construct, function_code);

        return function_sym;
    }

    static void create_reduction_function(OpenMP::Reduction* red,
            Nodecl::NodeclBase construct,
            TL::Type reduction_type,
            TL::Symbol& reduction_function,
            LoweringVisitor::reduction_map_t & reduction_map)
    {
        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
        {
            reduction_function = create_reduction_function_c(red, construct, reduction_map);
        }
        else if (IS_FORTRAN_LANGUAGE)
        {
            reduction_function = create_reduction_function_fortran(red, construct, reduction_map);
        }
        else
        {
            internal_error("Code unreachable", 0);
        }
    }

    static TL::Symbol create_initializer_function_c(
            OpenMP::Reduction* red,
            Nodecl::NodeclBase construct,
            LoweringVisitor::reduction_map_t& initializer_map)
    {
        LoweringVisitor::reduction_map_t::iterator it = initializer_map.find(red);
        if (it != initializer_map.end())
        {
            return it->second;
        }

        std::string fun_name;
        {
            std::stringstream ss;
            ss << "nanos_ini_" << red << "_" << simple_hash_str(construct.get_filename().c_str());
            fun_name = ss.str();
        }

        Nodecl::NodeclBase initializer = red->get_initializer().shallow_copy();
        if (initializer.is<Nodecl::StructuredValue>())
        {
            Nodecl::StructuredValue structured_value = initializer.as<Nodecl::StructuredValue>();
            if (structured_value.get_form().is<Nodecl::StructuredValueBraced>())
            {
                structured_value.set_form(Nodecl::StructuredValueCompoundLiteral::make());
            }
        }

        Source src;
        src << "void " << fun_name << "(" << as_type(red->get_type()) << "* omp_priv)"
            << "{"
            <<      "*omp_priv = " << as_expression(initializer) << ";"
            << "}"
            ;

        TL::Scope global_scope = construct.retrieve_context().get_global_scope();
        Nodecl::NodeclBase function_code = src.parse_global(global_scope);
        TL::Symbol function_sym = global_scope.get_symbol_from_name(fun_name);

        ERROR_CONDITION(!function_sym.is_valid(), "Symbol %s not found", fun_name.c_str());

        initializer_map[red] = function_sym;

        // As the initializer function is needed during the instantiation of
        // the task, this function should be inserted before the construct
        Nodecl::Utils::prepend_to_enclosing_top_level_location(construct,
                function_code);

        return function_sym;
    }

    static TL::Symbol create_initializer_function_fortran(
            OpenMP::Reduction* red,
            Nodecl::NodeclBase construct,
            LoweringVisitor::reduction_map_t & initializer_map)
    {
        LoweringVisitor::reduction_map_t::iterator it = initializer_map.find(red);
        if (it != initializer_map.end())
        {
            return it->second;
        }

        std::string fun_name;
        {
            std::stringstream ss;
            ss << "nanos_ini_" << red << "_" << simple_hash_str(construct.get_filename().c_str());
            fun_name = ss.str();
        }

        Nodecl::NodeclBase initializer = red->get_initializer().shallow_copy();
        Source src;

        src << "SUBROUTINE " << fun_name << "(omp_out)\n"
            <<    "IMPLICIT NONE\n"
            <<    as_type(red->get_type()) << " :: omp_out\n"
            <<    "omp_out = " << as_expression(initializer) << "\n"
            << "END SUBROUTINE " << fun_name << "\n";
        ;

        TL::Scope global_scope = construct.retrieve_context().get_global_scope();
        Nodecl::NodeclBase function_code = src.parse_global(global_scope);
        TL::Symbol function_sym = global_scope.get_symbol_from_name(fun_name);

        ERROR_CONDITION(!function_sym.is_valid(), "Symbol %s not found", fun_name.c_str());

        initializer_map[red] = function_sym;

        // As the initializer function is needed during the instantiation of
        // the task, this function should be inserted before the construct
        Nodecl::Utils::prepend_to_enclosing_top_level_location(construct,
                function_code);

        return function_sym;
    }

    static void create_initializer_function(OpenMP::Reduction* red,
            Nodecl::NodeclBase construct,
            TL::Type reduction_type,
            TL::Symbol& initializer_function,
            LoweringVisitor::reduction_map_t& initializer_map)
    {
        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
        {
            initializer_function = create_initializer_function_c(red, construct, initializer_map);
        }
        else if (IS_FORTRAN_LANGUAGE)
        {
            initializer_function = create_initializer_function_fortran(red, construct, initializer_map);
        }
        else
        {
            internal_error("Code unreachable", 0);
        }
    }

    class ReplaceReductionSymbols : public Nodecl::ExhaustiveVisitor<void>
    {
        const std::map<TL::Symbol, Nodecl::NodeclBase>& _reduction_symbol_to_nodecl_map;

        public:

        ReplaceReductionSymbols(const std::map<TL::Symbol, Nodecl::NodeclBase>& reduction_symbol_to_nodecl_map)
            : _reduction_symbol_to_nodecl_map(reduction_symbol_to_nodecl_map)
        {
        }

        void visit(const Nodecl::ObjectInit& node)
        {
            TL::Symbol sym = node.get_symbol();
            if (sym.get_value().is_null())
                return;

            walk(sym.get_value());
        }

        void visit(const Nodecl::Symbol& node)
        {
            TL::Symbol sym = node.get_symbol();
            std::map<TL::Symbol, Nodecl::NodeclBase>::const_iterator it = _reduction_symbol_to_nodecl_map.find(sym);
            if (it == _reduction_symbol_to_nodecl_map.end())
                return;

            node.replace(it->second.shallow_copy());
        }

    };

    void LoweringVisitor::handle_reductions_on_task(
            Nodecl::NodeclBase construct, OutlineInfo& outline_info, Nodecl::NodeclBase statements,
            Nodecl::NodeclBase& final_statements)
    {
        if (Nanos::Version::interface_is_at_least("task_reduction", 1000)
                || (Nanos::Version::interface_is_at_least("reduction_on_task", 1000)))
        {
            bool there_are_reductions_on_task = false;
            TL::Source reductions_stuff;
            std::map<TL::Symbol, std::string> reduction_symbols_map;

            TL::ObjectList<OutlineDataItem*> data_items = outline_info.get_data_items();
            for (TL::ObjectList<OutlineDataItem*>::iterator it = data_items.begin();
                    it != data_items.end();
                    it++)
            {
                if ((*it)->get_sharing() != OutlineDataItem::SHARING_CONCURRENT_REDUCTION)
                    continue;

                TL::Symbol sym = (*it)->get_symbol();

                there_are_reductions_on_task = true;

                std::string storage_name = (*it)->get_field_name() + "_storage";

                std::pair<TL::OpenMP::Reduction*, TL::Type> red_info_pair= (*it)->get_reduction_info();
                TL::OpenMP::Reduction* reduction_info = red_info_pair.first;
                TL::Type reduction_type = red_info_pair.second;

                TL::Symbol reduction_function;
                TL::Nanox::create_reduction_function(reduction_info,
                        construct,
                        reduction_type,
                        reduction_function,
                        _reduction_on_tasks_red_map);

                TL::Symbol initializer_function;
                create_initializer_function(reduction_info,
                        construct,
                        reduction_type,
                        initializer_function,
                        _reduction_on_tasks_ini_map);

                if (Nanos::Version::interface_is_at_least("task_reduction", 1000))
                {
                    reductions_stuff
                        << as_type(reduction_type.get_pointer_to()) << " " << storage_name << ";"
                        << "err = nanos_task_reduction_get_thread_storage("
                        <<      "(void *) &"<< (*it)->get_field_name() << ","
                        <<      "(void **) &" << storage_name << ");"
                        ;
                }
                else
                {
                    std::string cache_storage = (*it)->get_field_name() + "_cache_storage";
                    reductions_stuff
                        << as_type(reduction_type.get_pointer_to()) << " " << storage_name << ";"
                        << "nanos_TPRS_t* " << cache_storage << ";"
                        << "err = nanos_reduction_request_tprs("
                        <<      "(void *) &" << (*it)->get_field_name() << "," // target
                        <<      "sizeof(" << as_type(reduction_type) << "),"  // size
                        <<      "(void (*)(void *, void *)) &" << reduction_function.get_name() << "," // reducer
                        <<      "(void (*)(void *)) & " << initializer_function.get_name() << "," // initializer
                        <<      "(void (*)(void *, void *, void*)) reduction_reduce," // reducer atomic
                        <<      "(void (*)(void *)) reduction_flush," // flush
                        <<      "&" << cache_storage << ");"  // storage
                        ;

                    if (IS_FORTRAN_LANGUAGE)
                    {
                        // We need to convert a void* type into a pointer to the reduction type.
                        // As a void* in FORTRAN is represented as an INTEGER(8), we cannot do this
                        // conversion directly in the FORTRAN source. For this reason we introduce
                        // a new function that will be defined in a C file.
                        TL::Symbol func = TL::Nanox::get_function_ptr_conversion(
                                // Destination
                                reduction_type.get_pointer_to(),
                                // Origin
                                TL::Type::get_void_type().get_pointer_to(),
                                construct.retrieve_context());

                        reductions_stuff
                            << storage_name << " = " << func.get_name() <<"(" << cache_storage << "->storage);"
                            ;
                    }
                    else
                    {
                        reductions_stuff
                            << storage_name << " = "
                            <<   "(" << as_type(reduction_type.get_pointer_to()) << ")" << cache_storage << "->storage;"
                            ;
                    }
                }

                reduction_symbols_map[sym] = storage_name;
            }

            if (there_are_reductions_on_task)
            {
                {
                    Nodecl::NodeclBase placeholder;
                    TL::Source new_statements_src;
                    new_statements_src
                        << "{"
                        <<      "nanos_err_t err;"
                        <<      reductions_stuff
                        <<      statement_placeholder(placeholder)
                        << "}"
                        ;

                    if (IS_FORTRAN_LANGUAGE)
                        Source::source_language = SourceLanguage::C;

                    Nodecl::NodeclBase new_statements = new_statements_src.parse_statement(construct);

                    if (IS_FORTRAN_LANGUAGE)
                        Source::source_language = SourceLanguage::Current;


                    TL::Source final_statements_src;

                    TL::Scope new_scope = ReferenceScope(placeholder).get_scope();
                    std::map<TL::Symbol, Nodecl::NodeclBase> reduction_symbol_to_nodecl_map;
                    for (std::map<TL::Symbol, std::string>::iterator it = reduction_symbols_map.begin();
                            it != reduction_symbols_map.end();
                            ++it)
                    {
                        TL::Symbol reduction_sym = it->first;
                        std::string storage_name = it->second;
                        TL::Symbol storage_sym = new_scope.get_symbol_from_name(storage_name);
                        ERROR_CONDITION(!storage_sym.is_valid(), "This symbol is not valid\n", 0);
                        final_statements_src
                            << "if (" << storage_sym.get_name() << " == 0)"
                            << "{"
                            <<      storage_sym.get_name() << " = &" << reduction_sym.get_name() << ";"
                            << "}"
                            ;
                        Nodecl::NodeclBase deref_storage = Nodecl::Dereference::make(
                                storage_sym.make_nodecl(/* set_ref_type */ true, storage_sym.get_locus()),
                                storage_sym.get_type().points_to());

                        reduction_symbol_to_nodecl_map[reduction_sym] = deref_storage;
                    }

                    Nodecl::NodeclBase placeholder2;

                    ReplaceReductionSymbols visitor(reduction_symbol_to_nodecl_map);
                    Nodecl::NodeclBase copied_statements = statements.shallow_copy();
                    visitor.walk(copied_statements);

                    final_statements_src << as_statement(copied_statements);


                    if (IS_FORTRAN_LANGUAGE)
                        Source::source_language = SourceLanguage::C;

                    Nodecl::NodeclBase new_final_statements = final_statements_src.parse_statement(new_scope);

                    if (IS_FORTRAN_LANGUAGE)
                        Source::source_language = SourceLanguage::Current;

                    placeholder.replace(new_final_statements);

                    final_statements = new_statements;
                }

                {
                    TL::Source new_statements_src;
                    Nodecl::NodeclBase placeholder;
                    new_statements_src
                        << "{"
                        <<      "nanos_err_t err;"
                        <<      reductions_stuff
                        <<      statement_placeholder(placeholder)
                        << "}"
                        ;

                    if (IS_FORTRAN_LANGUAGE)
                        Source::source_language = SourceLanguage::C;

                    Nodecl::NodeclBase new_statements = new_statements_src.parse_statement(construct);

                    if (IS_FORTRAN_LANGUAGE)
                        Source::source_language = SourceLanguage::Current;


                    TL::Scope new_scope = ReferenceScope(placeholder).get_scope();
                    std::map<TL::Symbol, Nodecl::NodeclBase> reduction_symbol_to_nodecl_map;
                    for (std::map<TL::Symbol, std::string>::iterator it = reduction_symbols_map.begin();
                            it != reduction_symbols_map.end();
                            ++it)
                    {
                        TL::Symbol reduction_sym = it->first;
                        std::string storage_name = it->second;
                        TL::Symbol storage_sym = new_scope.get_symbol_from_name(storage_name);
                        ERROR_CONDITION(!storage_sym.is_valid(), "This symbol is not valid\n", 0);

                        Nodecl::NodeclBase deref_storage = Nodecl::Dereference::make(
                                storage_sym.make_nodecl(/* set_ref_type */ true, storage_sym.get_locus()),
                                storage_sym.get_type().points_to());

                        reduction_symbol_to_nodecl_map[reduction_sym] = deref_storage;
                    }

                    ReplaceReductionSymbols visitor(reduction_symbol_to_nodecl_map);
                    Nodecl::NodeclBase copied_statements = statements.shallow_copy();
                    visitor.walk(copied_statements);

                    placeholder.replace(copied_statements);
                    statements.replace(new_statements);
                }
            }
        }
    }

    void LoweringVisitor::register_reductions(Nodecl::NodeclBase construct, OutlineInfo& outline_info, TL::Source& src)
    {
        if (!Nanos::Version::interface_is_at_least("task_reduction", 1000))
            return;

        TL::ObjectList<OutlineDataItem*> data_items = outline_info.get_data_items();
        for (TL::ObjectList<OutlineDataItem*>::iterator it = data_items.begin();
                it != data_items.end();
                it++)
        {
            if ((*it)->get_sharing() != OutlineDataItem::SHARING_CONCURRENT_REDUCTION)
                continue;

            std::pair<TL::OpenMP::Reduction*, TL::Type> red_info_pair= (*it)->get_reduction_info();
            TL::OpenMP::Reduction* reduction_info = red_info_pair.first;
            TL::Type reduction_type = red_info_pair.second;

            TL::Symbol sym = (*it)->get_symbol();

            TL::Symbol reduction_function;
            TL::Nanox::create_reduction_function(reduction_info,
                    construct,
                    reduction_type,
                    reduction_function,
                    _reduction_on_tasks_red_map);

            TL::Symbol initializer_function;
            create_initializer_function(reduction_info,
                    construct,
                    reduction_type,
                    initializer_function,
                    _reduction_on_tasks_ini_map);

            src
                << "err = nanos_task_reduction_register("
                <<      "(void *) &" << (*it)->get_field_name() << ","    // target
                <<      "sizeof(" << as_type(reduction_type) << "),"    // size
                <<      "__alignof__(" << as_type(reduction_type) << "),"
                <<      "(void (*)(void *)) & " << initializer_function.get_name() << ","         // initializer
                <<      "(void (*)(void *, void *)) &" << reduction_function.get_name() << ");" // reducer
                ;
        }
    }

}}
