/*--------------------------------------------------------------------
  (C) Copyright 2017-2017 Barcelona Supercomputing Center
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


#include "codegen-fortran-llvm-expr.hpp"
#include "cxx-cexpr.h"

namespace Codegen
{

void FortranVisitorLLVMExpression::implement_builtin_call(
    const Nodecl::FunctionCall &node)
{
    Nodecl::NodeclBase called = node.get_called();
    Nodecl::List arguments = node.get_arguments().as<Nodecl::List>();

    ERROR_CONDITION(
        !called.is<Nodecl::Symbol>(), "We can only call functions", 0);
    TL::Symbol called_sym = called.get_symbol();

    BuiltinImpl &impl = builtin_impl[called_sym.get_name()];
    if (impl.use_constant && node.is_constant())
    {
        value = llvm_visitor->eval_constant(node.get_constant());
        return;
    }
    if (impl.func == nullptr)
    {
        internal_error("No implementation for builtin '%s'\n",
                       called_sym.get_name().c_str());
    }

    return (this->*(impl.func))(node);
}

void FortranVisitorLLVMExpression::builtin_xbound(
    const Nodecl::FunctionCall &node, bool is_lbound)
{
    Nodecl::List args = node.get_arguments().as<Nodecl::List>();
    Nodecl::NodeclBase array = args[0];
    Nodecl::NodeclBase dim = args.size() > 1 ? args[1] : Nodecl::NodeclBase();
    // KIND is already represented as the type of the node

    // FIXME: Implement array version of this
    if (dim.is_null())
        internal_error("Not yet implemented", 0);

    int dim_val = const_value_cast_to_4(dim.get_constant());
    TL::Type t = array.get_type().no_ref();
    ERROR_CONDITION(dim_val > t.fortran_rank(),
                    "Invalid dimension %d > %d",
                    dim_val,
                    t.fortran_rank());
    if (t.array_requires_descriptor())
    {
        llvm::Value *descriptor_addr = llvm_visitor->eval_expression(array);

        llvm::Value *xbound_field
            = is_lbound ?
                  llvm_visitor->array_descriptor_addr_dim_lower_bound(
                      descriptor_addr, dim_val - 1) :
                  llvm_visitor->array_descriptor_addr_dim_upper_bound(
                      descriptor_addr, dim_val - 1);

        value = llvm_visitor->ir_builder->CreateLoad(xbound_field);
    }
    else
    {
        for (int i = 0; i < dim_val; i++)
        {
            t = t.array_element();
        }
        ERROR_CONDITION(!t.is_fortran_array(), "Invalid type", 0);

        Nodecl::NodeclBase lower, upper;
        t.array_get_bounds(lower, upper);

        Nodecl::NodeclBase xbound = is_lbound ? lower : upper;

        value = llvm_visitor->eval_expression(xbound);
    }

    value = llvm_visitor->ir_builder->CreateZExtOrTrunc(
        value, llvm_visitor->get_llvm_type(node.get_type()));
}

void FortranVisitorLLVMExpression::builtin_lbound(
    const Nodecl::FunctionCall &node)
{
    return builtin_xbound(node, /* is_lbound */ true);
}

void FortranVisitorLLVMExpression::builtin_ubound(
    const Nodecl::FunctionCall &node)
{
    return builtin_xbound(node, /* is_lbound */ false);
}

std::map<std::string, FortranVisitorLLVMExpression::BuiltinImpl>
    FortranVisitorLLVMExpression::builtin_impl
    = { { "lbound", { &FortranVisitorLLVMExpression::builtin_lbound, true } },
        { "ubound", { &FortranVisitorLLVMExpression::builtin_ubound, true } } };
}
