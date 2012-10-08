/*--------------------------------------------------------------------
  (C) Copyright 2006-2012 Barcelona Supercomputing Center
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

#include "tl-devices.hpp"
#include "nanox-smp.hpp"

#include "tl-lowering-visitor.hpp"
#include "tl-source.hpp"
#include "tl-counters.hpp"
#include "tl-nodecl-utils.hpp"
#include "tl-outline-info.hpp"
#include "tl-replace.hpp"
#include "tl-compilerpipeline.hpp"

#include "codegen-phase.hpp"
#include "codegen-fortran.hpp"

#include "cxx-cexpr.h"
#include "fortran03-scope.h"
#include "fortran03-typeutils.h"
#include "fortran03-buildscope.h"

#include "cxx-profile.h"
#include "cxx-driver-utils.h"
#include <errno.h>
#include <string.h>

using TL::Source;

namespace TL { namespace Nanox {

    struct FortranExtraDeclsVisitor : Nodecl::ExhaustiveVisitor<void>
    {
        public:

            TL::ObjectList<TL::Symbol> extra_decl_sym;

            virtual void visit(const Nodecl::FunctionCall &function_call)
            {
                Nodecl::NodeclBase function_name = function_call.get_called();
                Nodecl::NodeclBase alternate_name = function_call.get_alternate_name();
                Nodecl::NodeclBase argument_seq = function_call.get_arguments();

                if (alternate_name.is_null())
                {
                    walk(function_name);
                }
                else
                {
                    walk(alternate_name);
                }

                walk(argument_seq);
            }

            virtual void visit(const Nodecl::Symbol &node_sym)
            {
                TL::Symbol sym = node_sym.get_symbol();
                if (sym.is_function())
                {
                    extra_decl_sym.insert(sym);
                }
            }

            virtual void visit(const Nodecl::StructuredValue &node)
            {
                TL::Type t = node.get_type();
                walk(node.get_items());

                if (t.is_named_class())
                {
                    extra_decl_sym.insert(t.get_symbol());
                }
            }
    };

    static std::string smp_outline_name(const std::string &task_name)
    {
        return "smp_" + task_name;
    }

    static TL::Symbol new_function_symbol_forward(
            TL::Symbol current_function,
            const std::string& function_name,
            OutlineInfo& outline_info)
    {
        Scope sc = current_function.get_scope();

        decl_context_t decl_context = sc.get_decl_context();
        decl_context_t function_context;

        if (IS_FORTRAN_LANGUAGE)
        {
            function_context = new_program_unit_context(decl_context);
        }
        else
        {
            function_context = new_function_context(decl_context);
            function_context = new_block_context(function_context);
        }

        TL::ObjectList<TL::Symbol> parameter_symbols, private_symbols;

        // Pointer to the real unpack function
        scope_entry_t* ptr_to_outline = ::new_symbol(function_context, function_context.current_scope,
                "outline_ptr");
        ptr_to_outline->kind = SK_VARIABLE;
        ptr_to_outline->type_information = fortran_choose_int_type_from_kind(CURRENT_CONFIGURATION->type_environment->sizeof_pointer);
        parameter_symbols.append(ptr_to_outline);

        TL::ObjectList<OutlineDataItem*> data_items = outline_info.get_data_items();
        for (TL::ObjectList<OutlineDataItem*>::iterator it = data_items.begin();
                it != data_items.end();
                it++)
        {
            TL::Symbol sym = (*it)->get_symbol();

            std::string name;
            if (sym.is_valid())
            {
                name = sym.get_name();
                if (IS_CXX_LANGUAGE
                        && name == "this")
                {
                    name = "this_";
                }
            }
            else
            {
                name = (*it)->get_field_name();
            }

            bool already_mapped = false;

            switch ((*it)->get_sharing())
            {
                case OutlineDataItem::SHARING_PRIVATE:
                    {
                        break;
                    }
                case OutlineDataItem::SHARING_SHARED_PRIVATE:
                case OutlineDataItem::SHARING_SHARED_CAPTURED_PRIVATE:
                    {
                        /* FALL THROUGH */
                    }
                case OutlineDataItem::SHARING_SHARED:
                case OutlineDataItem::SHARING_CAPTURE:
                case OutlineDataItem::SHARING_CAPTURE_ADDRESS:
                case OutlineDataItem::SHARING_REDUCTION:
                    {
                        switch ((*it)->get_item_kind())
                        {
                            case OutlineDataItem::ITEM_KIND_NORMAL:
                            case OutlineDataItem::ITEM_KIND_DATA_DIMENSION:
                                {
                                    scope_entry_t* private_sym = ::new_symbol(function_context, function_context.current_scope,
                                            name.c_str());
                                    private_sym->kind = SK_VARIABLE;
                                    if ((*it)->get_field_type().is_pointer()
                                            && (*it)->get_field_type().points_to().is_void())
                                    {
                                        // Preserve void*
                                        private_sym->type_information = (*it)
                                            ->get_field_type()
                                            .get_internal_type();
                                    }
                                    else
                                    {
                                        private_sym->type_information = (*it)
                                            ->get_field_type()
                                            .get_lvalue_reference_to()
                                            .get_internal_type();
                                    }
                                    private_sym->defined = private_sym->entity_specs.is_user_declared = 1;

                                    parameter_symbols.append(private_sym);
                                    break;
                                }
                            case OutlineDataItem::ITEM_KIND_DATA_ADDRESS:
                                {
                                    scope_entry_t* param_addr_sym = ::new_symbol(function_context, function_context.current_scope, 
                                            ("ptr_" + name).c_str());
                                    param_addr_sym->kind = SK_VARIABLE;
                                    param_addr_sym->type_information = ::get_pointer_type(::get_void_type());
                                    param_addr_sym->defined = param_addr_sym->entity_specs.is_user_declared = 1;

                                    parameter_symbols.append(param_addr_sym);

                                    scope_entry_t* private_sym = ::new_symbol(function_context, function_context.current_scope, name.c_str());
                                    private_sym->kind = SK_VARIABLE;
                                    if ((*it)->get_field_type().is_pointer()
                                            && (*it)->get_field_type().points_to().is_void())
                                    {
                                        // Preserve void*
                                        private_sym->type_information = (*it)
                                            ->get_field_type()
                                            .get_internal_type();
                                    }
                                    else
                                    {
                                        private_sym->type_information = (*it)
                                            ->get_field_type()
                                            .get_lvalue_reference_to()
                                            .get_internal_type();
                                    }
                                    private_sym->defined = private_sym->entity_specs.is_user_declared = 1;

                                    break;
                                }
                            default:
                                {
                                    internal_error("Code unreachable", 0);
                                }
                        }
                        break;
                    }
                default:
                    {
                        internal_error("Unexpected data sharing kind", 0);
                    }
            }
        }

        // // Update types of parameters (this is needed by VLAs)
        // for (TL::ObjectList<TL::Symbol>::iterator it = parameter_symbols.begin();
        //         it != parameter_symbols.end();
        //         it++)
        // {
        //     it->get_internal_symbol()->type_information =
        //         type_deep_copy(it->get_internal_symbol()->type_information,
        //                function_context,
        //                symbol_map->get_symbol_map());
        // }
        // Update types of privates (this is needed by VLAs)
        // for (TL::ObjectList<TL::Symbol>::iterator it = private_symbols.begin();
        //         it != private_symbols.end();
        //         it++)
        // {
        //     it->get_internal_symbol()->type_information =
        //         type_deep_copy(it->get_internal_symbol()->type_information,
        //                function_context,
        //                symbol_map->get_symbol_map());
        // }

        // Now everything is set to register the function
        scope_entry_t* new_function_sym = new_symbol(decl_context, decl_context.current_scope, function_name.c_str());
        new_function_sym->entity_specs.is_user_declared = 1;

        new_function_sym->kind = SK_FUNCTION;
        new_function_sym->file = "";
        new_function_sym->line = 0;

        // Make it static
        new_function_sym->entity_specs.is_static = 1;

        // Make it member if the enclosing function is member
        if (current_function.is_member())
        {
            new_function_sym->entity_specs.is_member = 1;
            new_function_sym->entity_specs.class_type = current_function.get_class_type().get_internal_type();

            new_function_sym->entity_specs.access = AS_PUBLIC;

            ::class_type_add_member(new_function_sym->entity_specs.class_type,
                    new_function_sym);
        }

        function_context.function_scope->related_entry = new_function_sym;
        function_context.block_scope->related_entry = new_function_sym;

        new_function_sym->related_decl_context = function_context;

        parameter_info_t* p_types = new parameter_info_t[parameter_symbols.size()];

        parameter_info_t* it_ptypes = &(p_types[0]);
        for (ObjectList<TL::Symbol>::iterator it = parameter_symbols.begin();
                it != parameter_symbols.end();
                it++, it_ptypes++)
        {
            scope_entry_t* param = it->get_internal_symbol();

            symbol_set_as_parameter_of_function(param, new_function_sym, new_function_sym->entity_specs.num_related_symbols);

            P_LIST_ADD(new_function_sym->entity_specs.related_symbols,
                    new_function_sym->entity_specs.num_related_symbols,
                    param);

            it_ptypes->is_ellipsis = 0;
            it_ptypes->nonadjusted_type_info = NULL;

            // FIXME - We should do all the remaining lvalue adjustments
            type_t* param_type = get_unqualified_type(param->type_information);
            it_ptypes->type_info = param_type;
        }

        type_t *function_type = get_new_function_type(
                get_void_type(),
                p_types,
                parameter_symbols.size());

        new_function_sym->type_information = function_type;

        delete[] p_types;

        return new_function_sym;
    }

    static TL::Symbol new_function_symbol_unpacked(
            TL::Symbol current_function,
            const std::string& function_name,
            OutlineInfo& outline_info,
            Nodecl::Utils::SymbolMap*& out_symbol_map)
    {
        Scope sc = current_function.get_scope();

        decl_context_t decl_context = sc.get_decl_context();
        decl_context_t function_context;

        if (IS_FORTRAN_LANGUAGE)
        {
            function_context = new_program_unit_context(decl_context);
        }
        else
        {
            function_context = new_function_context(decl_context);
            function_context = new_block_context(function_context);
        }

        // Create all the symbols and an appropiate mapping

        Nodecl::Utils::SimpleSymbolMap *symbol_map = new Nodecl::Utils::SimpleSymbolMap();

        TL::ObjectList<TL::Symbol> parameter_symbols, private_symbols;

        TL::ObjectList<OutlineDataItem*> data_items = outline_info.get_data_items();
        for (TL::ObjectList<OutlineDataItem*>::iterator it = data_items.begin();
                it != data_items.end();
                it++)
        {
            TL::Symbol sym = (*it)->get_symbol();

            std::string name;
            if (sym.is_valid())
            {
                name = sym.get_name();
                if (IS_CXX_LANGUAGE
                        && name == "this")
                {
                    name = "this_";
                }
            }
            else
            {
                name = (*it)->get_field_name();
            }

            bool already_mapped = false;

            switch ((*it)->get_sharing())
            {
                case OutlineDataItem::SHARING_PRIVATE:
                    {
                        scope_entry_t* private_sym = ::new_symbol(function_context, function_context.current_scope, name.c_str());
                        private_sym->kind = SK_VARIABLE;
                        private_sym->type_information = (*it)->get_in_outline_type().get_internal_type();
                        private_sym->defined = private_sym->entity_specs.is_user_declared = 1;

                        if (sym.is_valid())
                        {
                            symbol_map->add_map(sym, private_sym);

                            // Copy attributes that must be preserved
                            private_sym->entity_specs.is_allocatable = !sym.is_member() && sym.is_allocatable();
                        }

                        private_symbols.append(private_sym);
                        break;
                    }
                case OutlineDataItem::SHARING_SHARED_PRIVATE:
                case OutlineDataItem::SHARING_SHARED_CAPTURED_PRIVATE:
                    {
                        scope_entry_t* private_sym = ::new_symbol(function_context, function_context.current_scope, 
                                ("p_" + name).c_str());
                        private_sym->kind = SK_VARIABLE;
                        private_sym->type_information = (*it)->get_private_type().get_internal_type();
                        private_sym->defined = private_sym->entity_specs.is_user_declared = 1;

                        if (sym.is_valid())
                        {
                            symbol_map->add_map(sym, private_sym);

                            // Copy attributes that must be preserved
                            private_sym->entity_specs.is_allocatable = !sym.is_member() && sym.is_allocatable();

                            // We do not want it be mapped again
                            // in the fall-through branch
                            already_mapped = true;
                        }

                        private_symbols.append(private_sym);

                        /* FALL THROUGH */
                    }
                case OutlineDataItem::SHARING_SHARED:
                case OutlineDataItem::SHARING_CAPTURE:
                case OutlineDataItem::SHARING_CAPTURE_ADDRESS:
                    {
                        switch ((*it)->get_item_kind())
                        {
                            case OutlineDataItem::ITEM_KIND_NORMAL:
                            case OutlineDataItem::ITEM_KIND_DATA_DIMENSION:
                                {
                                    scope_entry_t* private_sym = ::new_symbol(function_context, function_context.current_scope,
                                            name.c_str());
                                    private_sym->kind = SK_VARIABLE;
                                    private_sym->type_information = (*it)->get_in_outline_type().get_internal_type();
                                    private_sym->defined = private_sym->entity_specs.is_user_declared = 1;


                                    if (sym.is_valid())
                                    {
                                        private_sym->entity_specs.is_optional = sym.is_optional();
                                        private_sym->entity_specs.is_allocatable =
                                            !sym.is_member() && sym.is_allocatable();
                                        if (!already_mapped)
                                        {
                                            symbol_map->add_map(sym, private_sym);
                                        }
                                    }

                                    parameter_symbols.append(private_sym);

                                    break;
                                }
                            case OutlineDataItem::ITEM_KIND_DATA_ADDRESS:
                                {
                                    scope_entry_t* param_addr_sym = ::new_symbol(function_context, function_context.current_scope, 
                                            ("ptr_" + name).c_str());
                                    param_addr_sym->kind = SK_VARIABLE;
                                    param_addr_sym->type_information = ::get_pointer_type(::get_void_type());
                                    param_addr_sym->defined = param_addr_sym->entity_specs.is_user_declared = 1;

                                    parameter_symbols.append(param_addr_sym);

                                    scope_entry_t* private_sym = ::new_symbol(function_context, function_context.current_scope, name.c_str());
                                    private_sym->kind = SK_VARIABLE;
                                    private_sym->type_information = (*it)->get_in_outline_type().get_internal_type();
                                    private_sym->defined = private_sym->entity_specs.is_user_declared = 1;

                                    // Properly initialize it
                                    nodecl_t nodecl_sym = nodecl_make_symbol(param_addr_sym, NULL, 0);
                                    nodecl_set_type(nodecl_sym, lvalue_ref(param_addr_sym->type_information));

                                    // This is doing
                                    //    T v = (T)ptr_v;
                                    private_sym->value = 
                                        nodecl_make_cast(
                                                nodecl_sym,
                                                private_sym->type_information,
                                                "C", NULL, 0);

                                    if (sym.is_valid()
                                            && !already_mapped)
                                    {
                                        symbol_map->add_map(sym, private_sym);
                                    }
                                    break;
                                }
                            default:
                                {
                                    internal_error("Code unreachable", 0);
                                }
                        }
                        break;
                    }
                case OutlineDataItem::SHARING_REDUCTION:
                    {
                        // Original reduced variable. Passed as we pass shared parameters
                        TL::Type param_type = (*it)->get_in_outline_type();
                        scope_entry_t* shared_reduction_sym = ::new_symbol(function_context, function_context.current_scope,
                                (*it)->get_field_name().c_str());
                        shared_reduction_sym->kind = SK_VARIABLE;
                        shared_reduction_sym->type_information = param_type.get_internal_type();
                        shared_reduction_sym->defined = shared_reduction_sym->entity_specs.is_user_declared = 1;
                        parameter_symbols.append(shared_reduction_sym);

                        shared_reduction_sym->entity_specs.is_allocatable = sym.is_valid()
                            && !sym.is_member()
                            && sym.is_allocatable();

                        // Private vector of partial reductions. This is a local pointer variable
                        // rdv stands for reduction vector
                        TL::Type private_reduction_vector_type = (*it)->get_private_type();
                        if (IS_C_LANGUAGE
                                || IS_CXX_LANGUAGE)
                        {
                            // T*
                            private_reduction_vector_type = private_reduction_vector_type.get_pointer_to();
                        }
                        else if (IS_FORTRAN_LANGUAGE)
                        {
                            // The type will be a pointer to a descripted array
                            private_reduction_vector_type = private_reduction_vector_type.get_array_to_with_descriptor(
                                    Nodecl::NodeclBase::null(),
                                    Nodecl::NodeclBase::null(),
                                    sc);
                            private_reduction_vector_type = private_reduction_vector_type.get_pointer_to();
                        }
                        else
                        {
                            internal_error("Code unreachable", 0);
                        }

                        scope_entry_t* private_reduction_vector_sym = ::new_symbol(function_context, function_context.current_scope,
                                ("rdv_" + name).c_str());
                        private_reduction_vector_sym->kind = SK_VARIABLE;
                        private_reduction_vector_sym->type_information = private_reduction_vector_type.get_internal_type();
                        private_reduction_vector_sym->defined = private_reduction_vector_sym->entity_specs.is_user_declared = 1;

                        // Local variable (rdp stands for reduction private)
                        // This variable must be initialized properly
                        scope_entry_t* private_sym = ::new_symbol(function_context, function_context.current_scope,
                                ("rdp_" + name).c_str());
                        private_sym->kind = SK_VARIABLE;
                        private_sym->type_information = (*it)->get_private_type().get_internal_type();
                        private_sym->defined = private_sym->entity_specs.is_user_declared = 1;

                        if (sym.is_valid())
                        {
                            symbol_map->add_map(sym, private_sym);
                        }

                        break;
                    }
                default:
                    {
                        internal_error("Unexpected data sharing kind", 0);
                    }
            }
        }

        // Update types of parameters (this is needed by VLAs)
        for (TL::ObjectList<TL::Symbol>::iterator it = parameter_symbols.begin();
                it != parameter_symbols.end();
                it++)
        {
            it->get_internal_symbol()->type_information =
                type_deep_copy(it->get_internal_symbol()->type_information,
                       function_context,
                       symbol_map->get_symbol_map());
        }
        // Update types of privates (this is needed by VLAs)
        for (TL::ObjectList<TL::Symbol>::iterator it = private_symbols.begin();
                it != private_symbols.end();
                it++)
        {
            it->get_internal_symbol()->type_information =
                type_deep_copy(it->get_internal_symbol()->type_information,
                       function_context,
                       symbol_map->get_symbol_map());
        }

        // Now everything is set to register the function
        scope_entry_t* new_function_sym = new_symbol(decl_context, decl_context.current_scope, function_name.c_str());
        new_function_sym->entity_specs.is_user_declared = 1;

        new_function_sym->kind = SK_FUNCTION;
        new_function_sym->file = "";
        new_function_sym->line = 0;

        // Make it static
        new_function_sym->entity_specs.is_static = 1;

        // Make it member if the enclosing function is member
        if (current_function.is_member())
        {
            new_function_sym->entity_specs.is_member = 1;
            new_function_sym->entity_specs.class_type = current_function.get_class_type().get_internal_type();

            new_function_sym->entity_specs.access = AS_PUBLIC;

            ::class_type_add_member(new_function_sym->entity_specs.class_type,
                    new_function_sym);
        }

        function_context.function_scope->related_entry = new_function_sym;
        function_context.block_scope->related_entry = new_function_sym;

        new_function_sym->related_decl_context = function_context;

        parameter_info_t* p_types = new parameter_info_t[parameter_symbols.size()];

        parameter_info_t* it_ptypes = &(p_types[0]);
        for (ObjectList<TL::Symbol>::iterator it = parameter_symbols.begin();
                it != parameter_symbols.end();
                it++, it_ptypes++)
        {
            scope_entry_t* param = it->get_internal_symbol();

            symbol_set_as_parameter_of_function(param, new_function_sym, new_function_sym->entity_specs.num_related_symbols);

            P_LIST_ADD(new_function_sym->entity_specs.related_symbols,
                    new_function_sym->entity_specs.num_related_symbols,
                    param);

            it_ptypes->is_ellipsis = 0;
            it_ptypes->nonadjusted_type_info = NULL;

            // FIXME - We should do all the remaining lvalue adjustments
            type_t* param_type = get_unqualified_type(param->type_information);
            it_ptypes->type_info = param_type;
        }

        type_t *function_type = get_new_function_type(
                get_void_type(),
                p_types,
                parameter_symbols.size());

        new_function_sym->type_information = function_type;

        delete[] p_types;

        out_symbol_map = symbol_map;
        return new_function_sym;
    }

    static TL::Symbol new_function_symbol(
            TL::Symbol current_function,
            const std::string& name,
            TL::Type return_type,
            ObjectList<std::string> parameter_names,
            ObjectList<TL::Type> parameter_types)
    {
        Scope sc = current_function.get_scope();

        // FIXME - Wrap
        decl_context_t decl_context = sc.get_decl_context();

        scope_entry_t* entry = new_symbol(decl_context, decl_context.current_scope, name.c_str());
        entry->entity_specs.is_user_declared = 1;

        entry->kind = SK_FUNCTION;
        entry->file = "";
        entry->line = 0;

        // Make it static
        entry->entity_specs.is_static = 1;

        // Make it member if the enclosing function is member
        if (current_function.is_member())
        {
            entry->entity_specs.is_member = 1;
            entry->entity_specs.class_type = current_function.get_class_type().get_internal_type();

            entry->entity_specs.access = AS_PUBLIC;

            ::class_type_add_member(entry->entity_specs.class_type, entry);
        }

        ERROR_CONDITION(parameter_names.size() != parameter_types.size(), "Mismatch between names and types", 0);

        decl_context_t function_context ;
        if (IS_FORTRAN_LANGUAGE)
        {
            function_context = new_program_unit_context(decl_context);
        }
        else
        {
            function_context = new_function_context(decl_context);
            function_context = new_block_context(function_context);
        }
        function_context.function_scope->related_entry = entry;
        function_context.block_scope->related_entry = entry;

        entry->related_decl_context = function_context;

        parameter_info_t* p_types = new parameter_info_t[parameter_types.size()];

        parameter_info_t* it_ptypes = &(p_types[0]);
        ObjectList<TL::Type>::iterator type_it = parameter_types.begin();
        for (ObjectList<std::string>::iterator it = parameter_names.begin();
                it != parameter_names.end();
                it++, it_ptypes++, type_it++)
        {
            scope_entry_t* param = new_symbol(function_context, function_context.current_scope, it->c_str());
            param->entity_specs.is_user_declared = 1;
            param->kind = SK_VARIABLE;
            param->file = "";
            param->line = 0;

            param->defined = 1;

            symbol_set_as_parameter_of_function(param, entry, entry->entity_specs.num_related_symbols);

            param->type_information = get_unqualified_type(type_it->get_internal_type());

            P_LIST_ADD(entry->entity_specs.related_symbols,
                    entry->entity_specs.num_related_symbols,
                    param);

            it_ptypes->is_ellipsis = 0;
            it_ptypes->nonadjusted_type_info = NULL;
            it_ptypes->type_info = get_indirect_type(param);
        }

        type_t *function_type = get_new_function_type(
                return_type.get_internal_type(),
                p_types,
                parameter_types.size());

        entry->type_information = function_type;

        delete[] p_types;

        return entry;
    }

    static void build_empty_body_for_function(
            TL::Symbol function_symbol,
            Nodecl::NodeclBase &function_code,
            Nodecl::NodeclBase &empty_stmt
            )
    {
        empty_stmt = Nodecl::EmptyStatement::make("", 0);
        Nodecl::List stmt_list = Nodecl::List::make(empty_stmt);

        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
        {
            Nodecl::CompoundStatement compound_statement =
                Nodecl::CompoundStatement::make(stmt_list,
                        /* destructors */ Nodecl::NodeclBase::null(),
                        "", 0);
            stmt_list = Nodecl::List::make(compound_statement);
        }

        Nodecl::NodeclBase context = Nodecl::Context::make(
                stmt_list,
                function_symbol.get_related_scope(), "", 0);

        function_symbol.get_internal_symbol()->defined = 1;

        function_code = Nodecl::FunctionCode::make(context,
                // Initializers
                Nodecl::NodeclBase::null(),
                // Internal functions
                Nodecl::NodeclBase::null(),
                function_symbol,
                "", 0);
    }

    Source DeviceSMP::emit_allocate_statement(TL::Symbol sym, int &lower_bound_index, int &upper_bound_index)
    {
        Source result;

        TL::Type t = sym.get_type();
        if (t.is_any_reference())
            t = t.references_to();

        struct Aux
        {
            static void aux_rec(Source &array_shape, TL::Type t, int rank, int current_rank,
                    int &lower_bound_index, int &upper_bound_index)
            {
                Source current_arg;
                if (t.is_array())
                {
                    aux_rec(array_shape, t.array_element(), rank-1, current_rank, lower_bound_index, upper_bound_index);

                    Source curent_arg;
                    Nodecl::NodeclBase lower, upper;
                    t.array_get_bounds(lower, upper);

                    if (lower.is_null())
                    {
                        current_arg << "mcc_lower_bound_" << lower_bound_index << ":";
                        lower_bound_index++;
                    }

                    if (upper.is_null())
                    {
                        current_arg << "mcc_upper_bound_" << upper_bound_index;
                        upper_bound_index++;
                    }

                    array_shape.append_with_separator(current_arg, ",");
                }
            }

            static void fill_array_shape(Source &array_shape, TL::Type t, int &lower_bound_index, int &upper_bound_index)
            {
                aux_rec(array_shape,
                        t, t.get_num_dimensions(), t.get_num_dimensions(),
                        lower_bound_index, upper_bound_index);
            }
        };

        Source array_shape;
        Aux::fill_array_shape(array_shape, t, lower_bound_index, upper_bound_index);

        result << "ALLOCATE(" << sym.get_name() << "(" << array_shape <<  "));\n"
            ;

        return result;
    }


    void DeviceSMP::create_outline(CreateOutlineInfo& info,
            Nodecl::NodeclBase& outline_placeholder,
            Nodecl::Utils::SymbolMap* &symbol_map)
    {
        //Unpack DTO
        const std::string& outline_name = smp_outline_name(info._outline_name);
        OutlineInfo& outline_info = info._outline_info;
        Nodecl::NodeclBase& original_statements = info._original_statements;
        TL::Symbol& arguments_struct = info._arguments_struct;

        TL::Symbol current_function = original_statements.retrieve_context().get_decl_context().current_scope->related_entry;

        if (current_function.is_nested_function())
        {
            if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
                running_error("%s: error: nested functions are not supported\n",
                        original_statements.get_locus().c_str());
            if (IS_FORTRAN_LANGUAGE)
                running_error("%s: error: internal subprograms are not supported\n",
                        original_statements.get_locus().c_str());
        }

        Source unpacked_arguments, cleanup_code, private_entities, extra_declarations;

        int lower_bound_index = 0;
        int upper_bound_index = 0;

        TL::ObjectList<OutlineDataItem*> data_items = outline_info.get_data_items();
        for (TL::ObjectList<OutlineDataItem*>::iterator it = data_items.begin();
                it != data_items.end();
                it++)
        {
            switch ((*it)->get_sharing())
            {
                case OutlineDataItem::SHARING_PRIVATE:
                    {
                        // Do nothing
                        if ((*it)->get_symbol().is_valid()
                                && (*it)->get_symbol().is_allocatable())
                        {
                            private_entities << emit_allocate_statement((*it)->get_symbol(), lower_bound_index, upper_bound_index);
                        }
                        break;
                    }
                case OutlineDataItem::SHARING_SHARED:
                case OutlineDataItem::SHARING_CAPTURE:
                case OutlineDataItem::SHARING_CAPTURE_ADDRESS:
                case OutlineDataItem::SHARING_SHARED_PRIVATE:
                case OutlineDataItem::SHARING_SHARED_CAPTURED_PRIVATE:
                    {
                        TL::Type param_type = (*it)->get_in_outline_type();

                        switch ((*it)->get_item_kind())
                        {
                            case OutlineDataItem::ITEM_KIND_NORMAL:
                            case OutlineDataItem::ITEM_KIND_DATA_DIMENSION:
                                {
                                    break;
                                }
                            case OutlineDataItem::ITEM_KIND_DATA_ADDRESS:
                                {
                                    param_type = TL::Type::get_void_type().get_pointer_to();

                                    break;
                                }
                            default:
                                {
                                    internal_error("Code unreachable", 0);
                                }
                        }

                        Source argument;
                        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
                        {
                            // Normal shared items are passed by reference from a pointer,
                            // derreference here
                            if (((*it)->get_sharing() == OutlineDataItem::SHARING_SHARED
                                        || (*it)->get_sharing() == OutlineDataItem::SHARING_SHARED_CAPTURED_PRIVATE
                                        || (*it)->get_sharing() == OutlineDataItem::SHARING_SHARED_PRIVATE)
                                    && (*it)->get_item_kind() == OutlineDataItem::ITEM_KIND_NORMAL
                                    && !(IS_CXX_LANGUAGE && (*it)->get_symbol().get_name() == "this"))
                            {
                                argument << "*(args." << (*it)->get_field_name() << ")";
                            }
                            // Any other thing is passed by value
                            else
                            {
                                argument << "args." << (*it)->get_field_name();
                            }

                            if (IS_CXX_LANGUAGE
                                    && (*it)->get_allocation_policy() == OutlineDataItem::ALLOCATION_POLICY_TASK_MUST_DESTROY)
                            {
                                internal_error("Not yet implemented: call the destructor", 0);
                            }
                        }
                        else if (IS_FORTRAN_LANGUAGE)
                        {
                            argument << "args % " << (*it)->get_field_name();

                            bool is_allocatable = (*it)->get_allocation_policy() & OutlineDataItem::ALLOCATION_POLICY_TASK_MUST_DEALLOCATE_ALLOCATABLE;
                            bool is_pointer = (*it)->get_allocation_policy() & OutlineDataItem::ALLOCATION_POLICY_TASK_MUST_DEALLOCATE_POINTER;

                            if (is_allocatable
                                    || is_pointer)
                            {
                                cleanup_code
                                    << "DEALLOCATE(args % " << (*it)->get_field_name() << ")\n"
                                    ;
                            }
                        }
                        else
                        {
                            internal_error("running error", 0);
                        }

                        if  ((*it)->get_sharing() == OutlineDataItem::SHARING_SHARED_CAPTURED_PRIVATE)
                        {
                            std::string name = (*it)->get_symbol().get_name();

                            private_entities
                                << "p_" << name << " = " << name << ";"
                                ;
                        }

                        unpacked_arguments.append_with_separator(argument, ", ");
                        break;
                    }
                case OutlineDataItem::SHARING_REDUCTION:
                    {
                        // Pass the original reduced variable as if it were a shared
                        Source argument;
                        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
                        {
                            argument << "*(args." << (*it)->get_field_name() << ")";
                        }
                        else if (IS_FORTRAN_LANGUAGE)
                        {
                            argument << "args % " << (*it)->get_field_name();
                        }
                        unpacked_arguments.append_with_separator(argument, ", ");

                        std::string name = (*it)->get_symbol().get_name();

                        private_entities
                            << "rdp_" << name << " = " << as_expression( (*it)->get_reduction_info()->get_identity()) << ";"
                            ;

                        break;
                    }
                default:
                    {
                        internal_error("Unexpected data sharing kind", 0);
                    }
            }
        }

        TL::Symbol unpacked_function, forward_function;
        if (IS_FORTRAN_LANGUAGE)
        {
            forward_function = new_function_symbol_forward(
                    current_function,
                    outline_name + "_forward",
                    outline_info);
            unpacked_function = new_function_symbol_unpacked(
                    current_function,
                    outline_name + "_unpack",
                    outline_info,
                    symbol_map);
        }
        else
        {
            unpacked_function = new_function_symbol_unpacked(
                    current_function,
                    outline_name + "_unpacked",
                    outline_info,
                    symbol_map);
        }

        outline_info.set_unpacked_function_symbol(unpacked_function);

        ObjectList<std::string> structure_name;
        structure_name.append("args");
        ObjectList<TL::Type> structure_type;
        structure_type.append(
                TL::Type(get_user_defined_type( arguments_struct.get_internal_symbol())).get_lvalue_reference_to()
                );

        TL::Symbol outline_function = new_function_symbol(
                current_function,
                outline_name,
                TL::Type::get_void_type(),
                structure_name,
                structure_type);

        if (IS_FORTRAN_LANGUAGE
                && current_function.is_in_module())
        {
            scope_entry_t* module_sym = current_function.in_module().get_internal_symbol();

            unpacked_function.get_internal_symbol()->entity_specs.in_module = module_sym;
            P_LIST_ADD(
                    module_sym->entity_specs.related_symbols,
                    module_sym->entity_specs.num_related_symbols,
                    unpacked_function.get_internal_symbol());

            unpacked_function.get_internal_symbol()->entity_specs.is_module_procedure = 1;

            outline_function.get_internal_symbol()->entity_specs.in_module = module_sym;
            P_LIST_ADD(
                    module_sym->entity_specs.related_symbols,
                    module_sym->entity_specs.num_related_symbols,
                    outline_function.get_internal_symbol());
            outline_function.get_internal_symbol()->entity_specs.is_module_procedure = 1;
        }

        Nodecl::NodeclBase unpacked_function_code, unpacked_function_body;
        build_empty_body_for_function(unpacked_function, 
                unpacked_function_code,
                unpacked_function_body);

        if (IS_FORTRAN_LANGUAGE)
        {
            // Copy FUNCTIONs and other local stuff
            symbol_map = new Nodecl::Utils::FortranProgramUnitSymbolMap(symbol_map,
                    current_function,
                    unpacked_function);

            // Replicate internal functions
            Nodecl::FunctionCode function_code = current_function.get_function_code().as<Nodecl::FunctionCode>();
            Nodecl::NodeclBase internal_functions = function_code.get_internal_functions();

            unpacked_function_code.as<Nodecl::FunctionCode>().set_internal_functions(
                    Nodecl::Utils::deep_copy(internal_functions, unpacked_function.get_related_scope(), *symbol_map));
        }

        Nodecl::Utils::append_to_top_level_nodecl(unpacked_function_code);

        Source unpacked_source;
        if (!IS_FORTRAN_LANGUAGE)
        {
            unpacked_source
                << "{";
        }
        unpacked_source
            << extra_declarations
            << private_entities
            << statement_placeholder(outline_placeholder)
            ;
        if (!IS_FORTRAN_LANGUAGE)
        {
            unpacked_source
                << "}";
        }

        // Fortran may require more symbols
        if (IS_FORTRAN_LANGUAGE)
        {
            FortranExtraDeclsVisitor fun_visitor;
            fun_visitor.walk(original_statements);

            extra_declarations
                << "IMPLICIT NONE\n";

            // Insert extra symbols
            TL::ReferenceScope ref_scope(unpacked_function_body);
            decl_context_t decl_context = ref_scope.get_scope().get_decl_context();

            for (ObjectList<Symbol>::iterator it = fun_visitor.extra_decl_sym.begin();
                    it != fun_visitor.extra_decl_sym.end();
                    it++)
            {
                // Insert the name in the context...
                TL::Scope sc = ref_scope.get_scope();
                ::insert_entry(decl_context.current_scope, it->get_internal_symbol());
            }

            // Copy USEd information
            scope_entry_t* original_used_modules_info
                = original_statements.retrieve_context().get_related_symbol().get_used_modules().get_internal_symbol();
            if (original_used_modules_info != NULL)
            {
                scope_entry_t* new_used_modules_info
                    = get_or_create_used_modules_symbol_info(decl_context);
                int i;
                for (i = 0 ; i< original_used_modules_info->entity_specs.num_related_symbols; i++)
                {
                    P_LIST_ADD(new_used_modules_info->entity_specs.related_symbols,
                            new_used_modules_info->entity_specs.num_related_symbols,
                            original_used_modules_info->entity_specs.related_symbols[i]);
                }
            }
        }
        else if (IS_CXX_LANGUAGE)
        {
            if (!unpacked_function.is_member())
            {
                Nodecl::NodeclBase nodecl_decl = Nodecl::CxxDecl::make(
                        /* optative context */ nodecl_null(),
                        unpacked_function,
                        original_statements.get_filename(),
                        original_statements.get_line());
                Nodecl::Utils::prepend_to_enclosing_top_level_location(original_statements, nodecl_decl);
            }
        }

        Nodecl::NodeclBase new_unpacked_body = unpacked_source.parse_statement(unpacked_function_body);
        unpacked_function_body.replace(new_unpacked_body);

        Nodecl::NodeclBase outline_function_code, outline_function_body;
        build_empty_body_for_function(outline_function,
                outline_function_code,
                outline_function_body);
        Nodecl::Utils::append_to_top_level_nodecl(outline_function_code);

        Source outline_src,
               instrument_before,
               instrument_after;

        if (IS_C_LANGUAGE || IS_CXX_LANGUAGE)
        {
            outline_src
                << "{"
                <<      instrument_before
                <<      outline_name << "_unpacked(" << unpacked_arguments << ");"
                <<      cleanup_code
                <<      instrument_after
                << "}"
                ;

            if (IS_CXX_LANGUAGE)
            {
                if (!outline_function.is_member())
                {
                    Nodecl::NodeclBase nodecl_decl = Nodecl::CxxDecl::make(
                            /* optative context */ nodecl_null(),
                            outline_function,
                            original_statements.get_filename(),
                            original_statements.get_line());
                    Nodecl::Utils::prepend_to_enclosing_top_level_location(original_statements, nodecl_decl);
                }
            }
        }
        else if (IS_FORTRAN_LANGUAGE)
        {
            Source outline_function_addr;

            outline_src
                << instrument_before << "\n"
                << "CALL " << outline_name << "_forward(" << outline_function_addr << unpacked_arguments << ")\n"
                << instrument_after << "\n"
                ;

            outline_function_addr << "LOC(" << unpacked_function.get_name() << ")";

            if (!unpacked_arguments.empty())
            {
                outline_function_addr << ", ";
            }

            TL::ReferenceScope ref_scope(outline_function_body);
            decl_context_t decl_context = ref_scope.get_scope().get_decl_context();

            // Copy USEd information
            scope_entry_t* original_used_modules_info
                = original_statements.retrieve_context().get_related_symbol().get_used_modules().get_internal_symbol();

            if (original_used_modules_info != NULL)
            {
                scope_entry_t* new_used_modules_info
                    = get_or_create_used_modules_symbol_info(decl_context);
                int i;
                for (i = 0 ; i< original_used_modules_info->entity_specs.num_related_symbols; i++)
                {
                    P_LIST_ADD(new_used_modules_info->entity_specs.related_symbols,
                            new_used_modules_info->entity_specs.num_related_symbols,
                            original_used_modules_info->entity_specs.related_symbols[i]);
                }
            }

            // Generate ancillary code in C
            add_forward_code_to_extra_c_code(outline_name, data_items, outline_placeholder);
        }
        else
        {
            internal_error("Code unreachable", 0);
        }

        if (instrumentation_enabled())
        {
            Source uf_name_id, uf_name_descr,
                   uf_location_id, uf_location_descr,
                   instrument_before_c, instrument_after_c;

            instrument_before_c
                << "static int nanos_funct_id_init = 0;"
                << "static nanos_event_key_t nanos_instr_uf_name_key = 0;"
                << "static nanos_event_value_t nanos_instr_uf_name_value = 0;"
                << "static nanos_event_key_t nanos_instr_uf_location_key = 0;"
                << "static nanos_event_value_t nanos_instr_uf_location_value = 0;"
                << "nanos_err_t err; "
                << "if (nanos_funct_id_init == 0)"
                << "{"
                <<    "err = nanos_instrument_get_key(\"user-funct-name\", &nanos_instr_uf_name_key);"
                <<    "if (err != NANOS_OK) nanos_handle_error(err);"
                <<    "err = nanos_instrument_register_value ( &nanos_instr_uf_name_value, \"user-funct-name\", "
                <<               uf_name_id << "," << uf_name_descr << ", 0);"
                <<    "if (err != NANOS_OK) nanos_handle_error(err);"

                <<    "err = nanos_instrument_get_key(\"user-funct-location\", &nanos_instr_uf_location_key);"
                <<    "if (err != NANOS_OK) nanos_handle_error(err);"
                <<    "err = nanos_instrument_register_value ( &nanos_instr_uf_location_value, \"user-funct-location\","
                <<               uf_location_id << "," << uf_location_descr << ", 0);"
                <<    "if (err != NANOS_OK) nanos_handle_error(err);"
                <<    "nanos_funct_id_init = 1;"
                << "}"
                << "nanos_event_t events_before[2];"
                << "events_before[0].type = NANOS_BURST_START;"
                << "events_before[0].key = nanos_instr_uf_name_key;"
                << "events_before[0].value = nanos_instr_uf_name_value;"
                << "events_before[1].type = NANOS_BURST_START;"
                << "events_before[1].key = nanos_instr_uf_location_key;"
                << "events_before[1].value = nanos_instr_uf_location_value;"
                << "err = nanos_instrument_events(2, events_before);"
                << "if (err != NANOS_OK) nanos_handle_error(err);"
                ;

            instrument_after_c
                << "nanos_event_t events_after[2];"
                << "events_after[0].type = NANOS_BURST_END;"
                << "events_after[0].key = nanos_instr_uf_name_key;"
                << "events_after[0].value = nanos_instr_uf_name_value;"
                << "events_after[1].type = NANOS_BURST_END;"
                << "events_after[1].key = nanos_instr_uf_location_key;"
                << "events_after[1].value = nanos_instr_uf_location_value;"
                << "err = nanos_instrument_events(2, events_after);"
                << "if (err != NANOS_OK) nanos_handle_error(err);"
                ;


            uf_name_id << uf_location_id;
            uf_location_id << "\"" << outline_name << ":" << original_statements.get_locus() << "\"";

            uf_name_descr << uf_location_descr;
            uf_location_descr
                << "\"Outline from '"
                << original_statements.get_locus()
                << "' in '" << outline_function.get_qualified_name() << "'\"";


            if (IS_FORTRAN_LANGUAGE)
                Source::source_language = SourceLanguage::C;

            Nodecl::NodeclBase instr_before = instrument_before_c.parse_statement(outline_function_body);
            Nodecl::NodeclBase instr_after = instrument_after_c.parse_statement(outline_function_body);

            if (IS_FORTRAN_LANGUAGE)
                Source::source_language = SourceLanguage::Current;

            instrument_before << as_statement(instr_before);
            instrument_after << as_statement(instr_after);
        }

        Nodecl::NodeclBase new_outline_body = outline_src.parse_statement(outline_function_body);
        outline_function_body.replace(new_outline_body);
    }

    void DeviceSMP::add_forward_code_to_extra_c_code(
            const std::string& outline_name,
            TL::ObjectList<OutlineDataItem*> data_items,
            Nodecl::NodeclBase parse_context)
    {
        Source ancillary_source, parameters;

        ancillary_source
            << "extern void " << outline_name << "_forward_" << "(";
        int num_data_items = data_items.size();
        if (num_data_items == 0)
        {
            ancillary_source << "void (*outline_fun)(void)";
        }
        else
        {
            ancillary_source << "void (*outline_fun)(";
            if (num_data_items == 0)
            {
                ancillary_source << "void";
            }
            else
            {
                for (int i = 0; i < num_data_items; i++)
                {
                    if (i > 0)
                    {
                        ancillary_source << ", ";
                    }
                    ancillary_source << "void *p" << i;
                }
            }
            ancillary_source << ")";

            for (int i = 0; i < num_data_items; i++)
            {
                ancillary_source << ", void *p" << i;
            }
        }
        ancillary_source << ")\n{\n"
            << "    extern int nanos_free(void*);\n"
            << "    extern int nanos_handle_error(int);\n\n"
            << "    outline_fun(";
        for (int i = 0; i < num_data_items; i++)
        {
            if (i > 0)
            {
                ancillary_source << ", ";
            }
            ancillary_source << "p" << i;
        }
        ancillary_source << ");\n";

        // Free all the allocated descriptors
        bool first = true;
        int i = 0;
        for (TL::ObjectList<OutlineDataItem*>::iterator it = data_items.begin();
                it != data_items.end();
                it++, i++)
        {
            OutlineDataItem &item (*(*it));

            if (item.get_symbol().is_valid()
                    && item.get_sharing() == OutlineDataItem::SHARING_SHARED)
            {
                TL::Type t = item.get_symbol().get_type();

                if (!item.get_symbol().is_allocatable()
                        && t.is_lvalue_reference()
                        && t.references_to().is_array()
                        && t.references_to().array_requires_descriptor())
                {
                    if (first)
                    {
                        ancillary_source << "   nanos_err_t err;\n";
                        first = false;
                    }

                    ancillary_source
                        << "    err = nanos_free(p" << i << ");\n"
                        << "    if (err != NANOS_OK) nanos_handle_error(err);\n"
                        ;
                }
            }
        }

        ancillary_source << "}\n\n";

        // Parse in C
        Source::source_language = SourceLanguage::C;

        Nodecl::List n = ancillary_source.parse_global(parse_context).as<Nodecl::List>();

        // Restore original source language (Fortran)
        Source::source_language = SourceLanguage::Current;

        // Figure a better way!
        for (Nodecl::List::iterator it = n.begin();
                it != n.end();
                it++)
        {
            _extra_c_code.push_back(*it);
        }
    }

    DeviceSMP::DeviceSMP()
        : DeviceProvider(/* device_name */ std::string("smp"))
    {
        set_phase_name("Nanox SMP support");
        set_phase_description("This phase is used by Nanox phases to implement SMP device support");
    }

    void DeviceSMP::pre_run(DTO& dto)
    {
    }

    void DeviceSMP::run(DTO& dto)
    {
        DeviceProvider::run(dto);
    }

    void DeviceSMP::get_device_descriptor(DeviceDescriptorInfo& info,
            Source &ancillary_device_description,
            Source &device_descriptor,
            Source &fortran_dynamic_init)
    {
        std::string outline_name = smp_outline_name(info._outline_name);
        if (!IS_FORTRAN_LANGUAGE)
        {
            ancillary_device_description
                << "static nanos_smp_args_t " << outline_name << "_args = {"
                << ".outline = (void(*)(void*))&" << outline_name
                << "};"
                ;
            device_descriptor
                << "{"
                << /* factory */ "&nanos_smp_factory, &" << outline_name << "_args"
                << "}"
                ;
        }
        else
        {
            ancillary_device_description
                << "static nanos_smp_args_t " << outline_name << "_args;"
                ;

            device_descriptor
                << "{"
                // factory, arg
                << "0, 0"
                << "}"
                ;

            fortran_dynamic_init
                << outline_name << "_args.outline = (void(*)(void*))&" << outline_name << ";"
                << "nanos_wd_const_data.devices[0].factory = &nanos_smp_factory;"
                << "nanos_wd_const_data.devices[0].arg = &" << outline_name << "_args;"
                ;
        }
    }

    void DeviceSMP::phase_cleanup(DTO& data_flow)
    {
        if (_extra_c_code.is_null())
            return;

        std::string original_filename = TL::CompilationProcess::get_current_file().get_filename();
        std::string new_filename = "aux_nanox_outline_file_" + original_filename  + ".c";

        FILE* ancillary_file = fopen(new_filename.c_str(), "w");
        if (ancillary_file == NULL)
        {
            running_error("%s: error: cannot open file '%s'. %s\n",
                    original_filename.c_str(),
                    new_filename.c_str(),
                    strerror(errno));
        }

        CURRENT_CONFIGURATION->source_language = SOURCE_LANGUAGE_C;

        compilation_configuration_t* configuration = ::get_compilation_configuration("auxcc");
        ERROR_CONDITION (configuration == NULL, "auxcc profile is mandatory when using Fortran", 0);

        // Make sure phases are loaded (this is needed for codegen)
        load_compiler_phases(configuration);

        TL::CompilationProcess::add_file(new_filename, "auxcc");

        ::mark_file_for_cleanup(new_filename.c_str());

        Codegen::CodegenPhase* phase = reinterpret_cast<Codegen::CodegenPhase*>(configuration->codegen_phase);
        phase->codegen_top_level(_extra_c_code, ancillary_file);

        CURRENT_CONFIGURATION->source_language = SOURCE_LANGUAGE_FORTRAN;

        fclose(ancillary_file);
        // Do not forget the clear the code for next files
        _extra_c_code.get_internal_nodecl() = nodecl_null();
    }

} }

EXPORT_PHASE(TL::Nanox::DeviceSMP);
