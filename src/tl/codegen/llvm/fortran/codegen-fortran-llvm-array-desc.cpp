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

#include "codegen-fortran-llvm.hpp"

namespace Codegen
{

llvm::Value *FortranLLVM::array_descriptor_addr_dim_data(
    llvm::Value *descriptor_addr, int dimension, const std::string &field)
{
    llvm::Type *descriptor_type
        = descriptor_addr->getType()->getPointerElementType();
    llvm::Type *descr_dim_type = gfortran_rt.descriptor_dimension.get();
    return ir_builder->CreateGEP(
        descriptor_addr,
        { get_integer_value_32(0),
          get_integer_value_32(fields[descriptor_type]["dim"]),
          get_integer_value_32(dimension),
          get_integer_value_32(fields[descr_dim_type][field]) });
}

llvm::Value *FortranLLVM::array_descriptor_addr_dim_lower_bound(
    llvm::Value *descriptor_addr, int dimension)
{
    return array_descriptor_addr_dim_data(
        descriptor_addr, dimension, "lower_bound");
}

llvm::Value *FortranLLVM::array_descriptor_addr_dim_upper_bound(
    llvm::Value *descriptor_addr, int dimension)
{
    return array_descriptor_addr_dim_data(
        descriptor_addr, dimension, "upper_bound");
}

llvm::Value *FortranLLVM::array_descriptor_addr_dim_stride(
    llvm::Value *descriptor_addr, int dimension)
{
    return array_descriptor_addr_dim_data(descriptor_addr, dimension, "stride");
}

llvm::Value *FortranLLVM::array_descriptor_addr_base_addr(
    llvm::Value *descriptor_addr)
{
    return gep_for_field(descriptor_addr->getType()->getPointerElementType(),
                         descriptor_addr,
                         { "base_addr" });
}

llvm::Value *FortranLLVM::array_descriptor_addr_offset(
    llvm::Value *descriptor_addr)
{
    return gep_for_field(descriptor_addr->getType()->getPointerElementType(),
                         descriptor_addr,
                         { "offset" });
}

llvm::Value *FortranLLVM::array_descriptor_addr_dtype(
    llvm::Value *descriptor_addr)
{
    return gep_for_field(descriptor_addr->getType()->getPointerElementType(),
                         descriptor_addr,
                         { "dtype" });
}

bool FortranLLVM::array_expression_will_use_unit_stride(Nodecl::NodeclBase expr)
{
    return expr.no_conv().is<Nodecl::Symbol>()
           && expr.no_conv().get_type().no_ref().is_fortran_array();
}

void FortranLLVM::allocate_array(const Nodecl::ArraySubscript &array_subscript)
{
    TrackLocation loc(this, array_subscript);

    Nodecl::NodeclBase array_name = array_subscript.get_subscripted();
    if (array_name.is<Nodecl::Dereference>())
        array_name = array_name.as<Nodecl::Dereference>().get_rhs();
    ERROR_CONDITION(!array_name.is<Nodecl::Symbol>(), "Invalid node", 0);
    TL::Symbol symbol = array_name.get_symbol();

    Nodecl::List array_sizes_tree
        = array_subscript.get_subscripts().as<Nodecl::List>();

    // In the tree the sizes are stored as subscripts using C layout, so we
    // have to reverse them here.
    TL::ObjectList<Nodecl::NodeclBase> array_sizes(array_sizes_tree.rbegin(),
                                                   array_sizes_tree.rend());

    if (symbol.is_allocatable()) // ALLOCATABLE
    {
    }
    else if (symbol.get_type().is_pointer()) // POINTER
    {
        internal_error("Not yet implemented", 0);
    }
    else
    {
        internal_error("Code unreachable", 0);
    }

    // Calculate first the size required to allocate the array storage
    llvm::Value *value_size = nullptr;
    for (TL::ObjectList<Nodecl::NodeclBase>::iterator size_it
         = array_sizes.begin();
         size_it != array_sizes.end();
         size_it++)
    {
        llvm::Value *extent = nullptr;
        if (size_it->is<Nodecl::Range>())
        {
            Nodecl::Range r = size_it->as<Nodecl::Range>();
            ERROR_CONDITION(
                !r.get_stride().is_null(), "No stride is allowed here", 0);
            extent = ir_builder->CreateAdd(
                ir_builder->CreateSub(
                    ir_builder->CreateZExtOrTrunc(
                        eval_expression(r.get_upper()), llvm_types.i64),
                    ir_builder->CreateZExtOrTrunc(
                        eval_expression(r.get_lower()), llvm_types.i64)),
                get_integer_value_64(1));
        }
        else
        {
            extent = ir_builder->CreateZExtOrTrunc(eval_expression(*size_it),
                                                   llvm_types.i64);
        }
        if (value_size == nullptr)
            value_size = extent;
        else
            value_size = ir_builder->CreateMul(value_size, extent);
    }

    // FIXME - Check whether the size overflows: gfortran does this

    // Check if the object has already been allocated. This simply checks if
    // base_addr is null
    llvm::Value *descriptor_addr = get_value(symbol);
    llvm::Value *field_addr_base_address
        = array_descriptor_addr_base_addr(descriptor_addr);
    llvm::Value *val_base_address
        = ir_builder->CreateLoad(field_addr_base_address);

    llvm::BasicBlock *already_allocated = llvm::BasicBlock::Create(
        llvm_context, "allocate.already_allocated", get_current_function());
    llvm::BasicBlock *not_allocated = llvm::BasicBlock::Create(
        llvm_context, "allocate.not_allocated", get_current_function());

    ir_builder->CreateCondBr(
        ir_builder->CreateICmpNE(
            ir_builder->CreatePtrToInt(val_base_address, llvm_types.i64),
            get_integer_value_64(0)),
        already_allocated,
        not_allocated);

    set_current_block(already_allocated);
    {
        std::stringstream ss;
        ss << "Array '" << symbol.get_name() << "' already allocated";
        gfortran_runtime_error(array_subscript.get_locus(), ss.str());
    }
    ir_builder->CreateBr(not_allocated); // This is to appease IR checker

    set_current_block(not_allocated);
    // Now allocate memory. Simply call malloc for this

    TL::Type base_elem
        = symbol.get_type().no_ref().fortran_array_base_element();
    llvm::Value *bytes_size
        = ir_builder->CreateMul(value_size, eval_sizeof_64(base_elem));

    llvm::Value *malloc_call
        = ir_builder->CreateCall(gfortran_rt.malloc.get(), { bytes_size });

    // FIXME - malloc can return NULL, this should be checked
    ir_builder->CreateStore(malloc_call, field_addr_base_address);

    fill_descriptor_info(symbol.get_type(),
                         /* descriptor addr  */ descriptor_addr,
                         /* base_address */ malloc_call,
                         array_sizes);
}

void FortranLLVM::fill_descriptor_info(
    int rank,
    TL::Type element_type,
    llvm::Value *descriptor_addr,
    llvm::Value *base_address,
    const std::vector<llvm::Value *> &lower_bounds,
    const std::vector<llvm::Value *> &upper_bounds,
    const std::vector<llvm::Value *> &strides)
{
    ERROR_CONDITION((unsigned)rank != lower_bounds.size(),
                    "Mismatch between rank and lower bounds",
                    0);
    ERROR_CONDITION((unsigned)rank != upper_bounds.size(),
                    "Mismatch between rank and upper bounds",
                    0);
    ERROR_CONDITION((unsigned)rank != strides.size(),
                    "Mismatch between rank and strides",
                    0);

    llvm::Value *field_addr_base_address
        = array_descriptor_addr_base_addr(descriptor_addr);
    llvm::Value *field_addr_offset
        = array_descriptor_addr_offset(descriptor_addr);

    llvm::Value *offset_value = nullptr;
    std::vector<llvm::Value *> descr_strides(rank, nullptr);
    for (int i = 0; i < rank; i++)
    {
        if (i == 0)
        {
            descr_strides[0] = strides[0];
            offset_value
                = ir_builder->CreateMul(lower_bounds[0], descr_strides[0]);
        }
        else
        {
            descr_strides[i] = ir_builder->CreateMul(
                strides[i],
                ir_builder->CreateAdd(
                    ir_builder->CreateSub(upper_bounds[i - 1],
                                          lower_bounds[i - 1]),
                    get_integer_value_64(1)));
            offset_value = ir_builder->CreateAdd(
                offset_value,
                ir_builder->CreateMul(lower_bounds[i], descr_strides[i]));
        }
    }

    // Negate offset_value
    offset_value = ir_builder->CreateSub(get_integer_value_64(0), offset_value);

    // base_address
    ir_builder->CreateStore(base_address, field_addr_base_address);
    // offset
    ir_builder->CreateStore(offset_value, field_addr_offset);

    // dtype
    int type_id = 0;
    // INTEGER          1
    // LOGICAL          2
    // REAL             3
    // COMPLEX          4
    // Any derived type 5
    // CHARACTER        6
    if (element_type.is_signed_integral())
        type_id = 1;
    else if (element_type.is_bool())
        type_id = 2;
    else if (element_type.is_floating_type())
        type_id = 3;
    else if (element_type.is_complex())
        type_id = 4;
    else if (element_type.is_class())
        type_id = 5;
    else if (element_type.is_fortran_character())
        type_id = 6;
    else
        internal_error("Uknown type '%s' for array\n",
                       print_declarator(element_type.get_internal_type()));

    int size_type = element_type.get_size();

    llvm::Value *dtype_value
        = get_integer_value_64(rank | (type_id << 3) | (size_type << 6));
    llvm::Value *field_addr_dtype
        = array_descriptor_addr_dtype(descriptor_addr);
    ir_builder->CreateStore(dtype_value, field_addr_dtype);

    // Ranks
    for (int i = 0; i < rank; i++)
    {
        ir_builder->CreateStore(
            descr_strides[i],
            array_descriptor_addr_dim_stride(descriptor_addr, i));
        ir_builder->CreateStore(
            lower_bounds[i],
            array_descriptor_addr_dim_lower_bound(descriptor_addr, i));
        ir_builder->CreateStore(
            upper_bounds[i],
            array_descriptor_addr_dim_upper_bound(descriptor_addr, i));
    }
}

void FortranLLVM::fill_descriptor_info(
    TL::Type array_type,
    llvm::Value *descriptor_addr,
    llvm::Value *base_address,
    const TL::ObjectList<Nodecl::NodeclBase> &array_sizes)
{
    int rank = array_type.fortran_rank();
    ERROR_CONDITION(array_sizes.size() != (unsigned)rank,
                    "Mismatch between rank and sizes",
                    0);

    std::vector<llvm::Value *> lower_bounds;
    lower_bounds.reserve(rank);
    std::vector<llvm::Value *> upper_bounds;
    upper_bounds.reserve(rank);
    std::vector<llvm::Value *> strides;
    strides.reserve(rank);

    for (TL::ObjectList<Nodecl::NodeclBase>::const_iterator it
         = array_sizes.begin();
         it != array_sizes.end();
         it++)
    {
        if (it->is<Nodecl::Range>())
        {
            Nodecl::Range r = it->as<Nodecl::Range>();

            lower_bounds.push_back(ir_builder->CreateZExtOrTrunc(
                eval_expression(r.get_lower()), llvm_types.i64));
            upper_bounds.push_back(ir_builder->CreateZExtOrTrunc(
                eval_expression(r.get_upper()), llvm_types.i64));
            strides.push_back(ir_builder->CreateZExtOrTrunc(
                eval_expression(r.get_stride()), llvm_types.i64));
        }
        else
        {
            lower_bounds.push_back(get_integer_value_64(1));
            upper_bounds.push_back(ir_builder->CreateZExtOrTrunc(
                eval_expression(*it), llvm_types.i64));
            strides.push_back(get_integer_value_64(1));
        }
    }

    fill_descriptor_info(rank,
                         array_type.fortran_array_base_element(),
                         descriptor_addr,
                         base_address,
                         lower_bounds,
                         upper_bounds,
                         strides);
}

void FortranLLVM::fill_descriptor_info(TL::Type array_type,
                                       llvm::Value *descriptor_addr,
                                       llvm::Value *base_address)
{
    std::vector<llvm::Value *> lower_bounds;
    std::vector<llvm::Value *> upper_bounds;
    std::vector<llvm::Value *> strides;

    TL::Type t = array_type;
    while (t.is_fortran_array())
    {
        if (t.array_is_region())
        {
            Nodecl::NodeclBase lb, ub, stride;
            t.array_get_region_bounds(lb, ub, stride);
            lower_bounds.push_back(ir_builder->CreateZExtOrTrunc(
                eval_expression(lb), llvm_types.i64));
            upper_bounds.push_back(ir_builder->CreateZExtOrTrunc(
                eval_expression(ub), llvm_types.i64));
            strides.push_back(ir_builder->CreateZExtOrTrunc(
                eval_expression(stride), llvm_types.i64));
        }
        else
        {
            Nodecl::NodeclBase lb, ub;
            t.array_get_bounds(lb, ub);
            lower_bounds.push_back(ir_builder->CreateZExtOrTrunc(
                eval_expression(lb), llvm_types.i64));
            upper_bounds.push_back(ir_builder->CreateZExtOrTrunc(
                eval_expression(ub), llvm_types.i64));
            strides.push_back(get_integer_value_64(1));
        }

        t = t.array_element();
    }

    return fill_descriptor_info(array_type.fortran_rank(),
                                array_type.fortran_array_base_element(),
                                descriptor_addr,
                                base_address,
                                lower_bounds,
                                upper_bounds,
                                strides);
}
}
