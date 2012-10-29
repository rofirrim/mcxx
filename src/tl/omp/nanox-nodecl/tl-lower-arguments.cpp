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

#include "tl-lowering-visitor.hpp"
#include "tl-outline-info.hpp"
#include "tl-source.hpp"
#include "tl-counters.hpp"

#include "fortran03-typeutils.h"

namespace TL { namespace Nanox {

    TL::Symbol LoweringVisitor::declare_argument_structure(OutlineInfo& outline_info, Nodecl::NodeclBase construct)
    {
        // Come up with a unique name
        Counter& counter = CounterManager::get_counter("nanos++-struct");
        std::string structure_name;

        std::stringstream ss;
        ss << "nanos_args_" << (int)counter << "_t";
        counter++;

        if (IS_C_LANGUAGE)
        {
            // We need an extra 'struct '
            structure_name = "struct " + ss.str();
        }
        else
        {
            structure_name = ss.str();
        }

        // FIXME - Wrap lots of things
        TL::Scope sc(construct.retrieve_context()
                .get_decl_context()
                .namespace_scope->related_entry // SK_NAMESPACE
                ->decl_context);

        TL::Symbol related_symbol = construct.retrieve_context().get_related_symbol();
        if (related_symbol.is_member())
        {
            // Class scope
            sc = ::class_type_get_inner_context(related_symbol.get_class_type().get_internal_type());
        }
        else if (related_symbol.is_in_module())
        {
            // Scope of the module
            sc = related_symbol.in_module().get_related_scope();
        }

        TL::Symbol new_class_symbol = sc.new_symbol(structure_name);
        new_class_symbol.get_internal_symbol()->kind = SK_CLASS;
        new_class_symbol.get_internal_symbol()->entity_specs.is_user_declared = 1;

        type_t* new_class_type = get_new_class_type(sc.get_decl_context(), TT_STRUCT);
        decl_context_t class_context = new_class_context(sc.get_decl_context(), new_class_symbol.get_internal_symbol());
        TL::Scope class_scope(class_context);

        class_type_set_inner_context(new_class_type, class_context);

        new_class_symbol.get_internal_symbol()->type_information = new_class_type;

        TL::ObjectList<OutlineDataItem*> data_items = outline_info.get_data_items();
        for (TL::ObjectList<OutlineDataItem*>::iterator it = data_items.begin();
                it != data_items.end();
                it++)
        {
            // Privates are ignored here
            if ((*it)->get_sharing() == OutlineDataItem::SHARING_PRIVATE)
                continue;

            TL::Symbol field = class_scope.new_symbol((*it)->get_field_name());
            field.get_internal_symbol()->kind = SK_VARIABLE;
            field.get_internal_symbol()->entity_specs.is_user_declared = 1;

            TL::Type field_type = (*it)->get_field_type();

            if (IS_CXX_LANGUAGE || IS_C_LANGUAGE)
            {
                if (field_type.is_const())
                {
                    field_type = field_type.get_unqualified_type();
                }
            }
            field.get_internal_symbol()->type_information = field_type.get_internal_type();

            field.get_internal_symbol()->entity_specs.is_member = 1;
            field.get_internal_symbol()->entity_specs.class_type = ::get_user_defined_type(new_class_symbol.get_internal_symbol());
            field.get_internal_symbol()->entity_specs.access = AS_PUBLIC;

            field.get_internal_symbol()->file = uniquestr(construct.get_filename().c_str());
            field.get_internal_symbol()->line = construct.get_line();

            // Language specific parts
            if (IS_FORTRAN_LANGUAGE)
            {
                if (((*it)->get_allocation_policy() & OutlineDataItem::ALLOCATION_POLICY_TASK_MUST_DEALLOCATE_ALLOCATABLE)
                        == OutlineDataItem::ALLOCATION_POLICY_TASK_MUST_DEALLOCATE_ALLOCATABLE)
                {
                    field.get_internal_symbol()->entity_specs.is_allocatable = 1;
                }
            }

            class_type_add_member(new_class_type, field.get_internal_symbol());
        }

        nodecl_t nodecl_output = nodecl_null();
        finish_class_type(new_class_type,
                ::get_user_defined_type(new_class_symbol.get_internal_symbol()),
                sc.get_decl_context(),
                construct.get_filename().c_str(),
                construct.get_line(),
                &nodecl_output);
        set_is_complete_type(new_class_type, /* is_complete */ 1);
        set_is_complete_type(get_actual_class_type(new_class_type), /* is_complete */ 1);

        if (!nodecl_is_null(nodecl_output))
        {
            std::cerr << "FIXME: finished class issues nonempty nodecl" << std::endl;
        }

        if (related_symbol.is_member())
        {
            new_class_symbol.get_internal_symbol()->entity_specs.is_member = 1;
            new_class_symbol.get_internal_symbol()->entity_specs.class_type 
                = related_symbol.get_class_type().get_internal_type();
            new_class_symbol.get_internal_symbol()->entity_specs.access = AS_PUBLIC;

            new_class_symbol.get_internal_symbol()->entity_specs.is_defined_inside_class_specifier = 
                related_symbol.get_internal_symbol()->entity_specs.is_defined_inside_class_specifier;

            ::class_type_add_member_before(
                    related_symbol.get_class_type().get_internal_type(), 
                    related_symbol.get_internal_symbol(),
                    new_class_symbol.get_internal_symbol());
        }
        else if (related_symbol.is_in_module())
        {
            // Add the newly created argument as a structure
            TL::Symbol module = construct.retrieve_context().get_related_symbol().in_module();

            new_class_symbol.get_internal_symbol()->entity_specs.in_module = module.get_internal_symbol();
            new_class_symbol.get_internal_symbol()->entity_specs.access = AS_PRIVATE;

            P_LIST_ADD(
                    module.get_internal_symbol()->entity_specs.related_symbols,
                    module.get_internal_symbol()->entity_specs.num_related_symbols,
                    new_class_symbol.get_internal_symbol());
        }

        CXX_LANGUAGE()
        {
            Nodecl::NodeclBase nodecl_decl = Nodecl::CxxDef::make(
                    /* optative context */ nodecl_null(),
                    new_class_symbol,
                    construct.get_filename(),
                    construct.get_line());
            Nodecl::Utils::prepend_to_enclosing_top_level_location(construct, nodecl_decl);
        }

        return new_class_symbol;
    }
} }
