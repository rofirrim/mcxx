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
#include "fortran03-mangling.h"

namespace Codegen
{

void FortranVisitorLLVMExpression::visit(const Nodecl::Symbol &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);
    value = llvm_visitor->get_value(node.get_symbol());
}

llvm::Value *FortranVisitorLLVMExpression::scalar_conversion(
    TL::Type dest, TL::Type orig, llvm::Value *value_nest)
{
    ERROR_CONDITION(dest.no_ref().is_fortran_array(), "Invalid type", 0);
    TL::Type real_dest = dest;
    TL::Type real_orig = orig;

    llvm::Value *result = value_nest;

    bool does_load = orig.is_any_reference() && !dest.is_any_reference();
    bool only_load = does_load && orig.no_ref().is_same_type(dest);
    bool needs_temporary = !orig.is_any_reference() && dest.is_any_reference();
    // bool does_rebinding = orig.is_any_reference() && dest.is_any_reference();
    if (does_load)
    {
        result = llvm_visitor->ir_builder->CreateLoad(result);
        if (only_load)
            return result;
    }

    dest = dest.no_ref();
    orig = orig.no_ref();

    // Arithmetic conversions
    if (dest.is_signed_integral() && orig.is_signed_integral())
    {
        result = llvm_visitor->ir_builder->CreateSExtOrTrunc(
            result, llvm_visitor->get_llvm_type(dest));
    }
    else if (dest.is_floating_type() && orig.is_floating_type())
    {
        // float < double < float128
        if (dest.get_size() > orig.get_size())
        {
            result = llvm_visitor->ir_builder->CreateFPExt(
                result, llvm_visitor->get_llvm_type(dest));
        }
        else if (dest.get_size() < orig.get_size())
        {
            result = llvm_visitor->ir_builder->CreateFPTrunc(
                result, llvm_visitor->get_llvm_type(dest));
        }
        else
        {
            internal_error("Code unreachable", 0);
        }
    }
    else if (dest.is_floating_type() && orig.is_signed_integral())
    {
        result = llvm_visitor->ir_builder->CreateSIToFP(
            result, llvm_visitor->get_llvm_type(dest));
    }
    else if (dest.is_signed_integral() && orig.is_floating_type())
    {
        result = llvm_visitor->ir_builder->CreateFPToSI(
            result, llvm_visitor->get_llvm_type(dest));
    }
    else
    {
        internal_error("Unhandled implicit conversion from '%s' to '%s'\n",
                       print_declarator(real_orig.get_internal_type()),
                       print_declarator(real_dest.get_internal_type()));
    }

    if (needs_temporary)
    {
        result = llvm_visitor->make_temporary(result);
    }

    return result;
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Conversion &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    TL::Type dest = node.get_type();
    TL::Type orig = node.get_nest().get_type();

    llvm::Value *nest_value = llvm_visitor->eval_expression(node.get_nest());

    if (dest.is_fortran_array() && orig.no_ref().is_fortran_array())
    {
        // We do not represent values of array, so we let the address
        // pass-through
        // FIXME: Arrays with region that are demoted to non-region arrays
        ERROR_CONDITION(
            !dest.array_element().is_same_type(orig.no_ref().array_element()),
            "array-wise value conversions not implemented yet",
            0);
        value = nest_value;
    }
    else if (dest.is_fortran_character() && orig.is_any_reference()
             && orig.no_ref().is_fortran_character())
    {
        // We do not represent values of array, so we let the address
        // pass-through
        value = nest_value;
    }
    else if (dest.is_fortran_array() && !orig.no_ref().is_fortran_array())
    {
        value = scalar_conversion(dest.array_base_element(), orig, nest_value);
    }
    else
    {
        value = scalar_conversion(dest, orig, nest_value);
    }
}

bool FortranVisitorLLVMExpression::is_scalar_to_array(
    const Nodecl::NodeclBase &n)
{
    return n.get_type().is_fortran_array()
           && !n.no_conv().get_type().no_ref().is_fortran_array();
}

void FortranVisitorLLVMExpression::create_loop_header_for_array_op(
    Nodecl::NodeclBase expr,
    TL::Type t,
    llvm::Value *addr,
    /* out */
    LoopInfoOp &loop_info_op)
{
    t = t.no_ref();
    if (t.is_pointer())
        t = t.points_to();
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type!\n", 0);
    int rank = t.fortran_rank();

    for (int i = 0; i < rank; i++)
    {
        llvm::BasicBlock *block_check
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "array_op.check",
                                       llvm_visitor->get_current_function());
        loop_info_op.block_check.push_back(block_check);

        llvm::BasicBlock *block_body
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "array_op.body",
                                       llvm_visitor->get_current_function());
        loop_info_op.block_body.push_back(block_body);

        llvm::BasicBlock *block_end
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "array_op.end",
                                       llvm_visitor->get_current_function());
        loop_info_op.block_end.push_back(block_end);

        llvm::Value *idx_var = llvm_visitor->ir_builder->CreateAlloca(
            llvm_visitor->llvm_types.i64);
        loop_info_op.idx_var.push_back(idx_var);

        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->get_integer_value_64(0), idx_var);

        llvm::Value *vstride = nullptr;
        llvm::Value *vupper = nullptr;
        llvm::Value *vlower = nullptr;
        if (t.array_requires_descriptor())
        {
            ERROR_CONDITION(
                addr == NULL,
                "Invalid address, required for arrays with descriptor\n",
                0);
            if (llvm_visitor->array_expression_will_use_unit_stride(expr))
                vstride = llvm_visitor->get_integer_value_64(1);
            else
                vstride = llvm_visitor->ir_builder->CreateLoad(
                    llvm_visitor->array_descriptor_addr_dim_stride(addr, i));
            vupper = llvm_visitor->ir_builder->CreateLoad(
                llvm_visitor->array_descriptor_addr_dim_upper_bound(addr, i));
            vlower = llvm_visitor->ir_builder->CreateLoad(
                llvm_visitor->array_descriptor_addr_dim_lower_bound(addr, i));
        }
        else if (t.array_is_region())
        {
            Nodecl::NodeclBase lower, upper, stride;
            t.array_get_region_bounds(lower, upper, stride);

            vupper = llvm_visitor->eval_expression(upper);
            vlower = llvm_visitor->eval_expression(lower);
            vstride = llvm_visitor->eval_expression(stride);
        }
        else
        {
            Nodecl::NodeclBase lower, upper;
            t.array_get_bounds(lower, upper);

            vupper = llvm_visitor->eval_expression(upper);
            vlower = llvm_visitor->eval_expression(lower);
            vstride = llvm_visitor->get_integer_value_64(1);
        }

        llvm::Value *loop_upper = llvm_visitor->ir_builder->CreateSDiv(
            llvm_visitor->ir_builder->CreateAdd(
                llvm_visitor->ir_builder->CreateSub(vupper, vlower), vstride),
            vstride);

        llvm_visitor->ir_builder->CreateBr(block_check);
        llvm_visitor->set_current_block(block_check);

        llvm::Value *vcheck = llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), loop_upper);
        llvm_visitor->ir_builder->CreateCondBr(vcheck, block_body, block_end);

        llvm_visitor->set_current_block(block_body);

        t = t.array_element();
    }
}

void FortranVisitorLLVMExpression::create_loop_footer_for_array_op(
    TL::Type t, const LoopInfoOp &loop_info_op)
{
    t = t.no_ref();
    if (t.is_pointer())
        t = t.points_to();
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type!\n", 0);
    int rank = t.fortran_rank();

    // Sanity check
    ERROR_CONDITION((int)loop_info_op.idx_var.size() != rank
                        || (int)loop_info_op.block_check.size() != rank
                        || (int)loop_info_op.block_body.size() != rank
                        || (int)loop_info_op.block_end.size() != rank,
                    "Inconsistency between rank and loop info",
                    0);


    for (int i = rank - 1; i >= 0; i--)
    {
        llvm::Value *idx_var = loop_info_op.idx_var[i];
        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->ir_builder->CreateAdd(
                llvm_visitor->ir_builder->CreateLoad(idx_var),
                llvm_visitor->get_integer_value_64(1)),
            idx_var);

        llvm::BasicBlock *block_check = loop_info_op.block_check[i];
        llvm_visitor->ir_builder->CreateBr(block_check);

        llvm::BasicBlock *block_end = loop_info_op.block_end[i];
        llvm_visitor->set_current_block(block_end);
    }
}

// FIXME: Integrate with address_array_ith_element_via_pointer_arithmetic
llvm::Value *FortranVisitorLLVMExpression::
    address_array_ith_element_via_descriptor(
        const Nodecl::NodeclBase &array,
        TL::Type t,
        llvm::Value *descr_address,
        const std::vector<llvm::Value *> indexes)
{
    bool unit_stride
        = llvm_visitor->array_expression_will_use_unit_stride(array);
    int rank = t.fortran_rank();

    // Horner
    std::vector<llvm::Value *> offset_list, size_list;
    offset_list.reserve(indexes.size());
    size_list.reserve(indexes.size());

    for (int i = 0; i < rank; i++)
    {
        llvm::Value *offset = indexes[i];
        if (!unit_stride)
        {
            llvm::Value *val_stride = llvm_visitor->ir_builder->CreateLoad(
                llvm_visitor->array_descriptor_addr_dim_stride(descr_address,
                                                               i));
            offset = llvm_visitor->ir_builder->CreateMul(offset, val_stride);
        }
        offset_list.push_back(offset);

        llvm::Value *val_lower = llvm_visitor->ir_builder->CreateLoad(
            llvm_visitor->array_descriptor_addr_dim_lower_bound(descr_address,
                                                                i));
        llvm::Value *val_upper = llvm_visitor->ir_builder->CreateLoad(
            llvm_visitor->array_descriptor_addr_dim_upper_bound(descr_address,
                                                                i));

        llvm::Value *val_size = llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateSub(val_upper, val_lower),
            llvm_visitor->get_integer_value_64(1));
        size_list.push_back(val_size);

        t = t.array_element();
    }

    std::vector<llvm::Value *>::iterator it_offsets = offset_list.begin();
    std::vector<llvm::Value *>::iterator it_sizes = size_list.begin();

    llvm::Value *val_addr = *it_offsets;
    it_offsets++;
    it_sizes++;

    while (it_offsets != offset_list.end() && it_sizes != size_list.end())
    {
        val_addr = llvm_visitor->ir_builder->CreateAdd(
            *it_offsets,
            llvm_visitor->ir_builder->CreateMul(*it_sizes, val_addr));

        it_offsets++;
        it_sizes++;
    }

    ERROR_CONDITION(it_offsets != offset_list.end()
                        || it_sizes != size_list.end(),
                    "Lists do not match",
                    0);

    ERROR_CONDITION(t.is_fortran_array(), "Should not be an array here", 0);

    llvm::Value *base_address = llvm_visitor->ir_builder->CreateLoad(
        llvm_visitor->array_descriptor_addr_base_addr(descr_address));

    base_address = llvm_visitor->ir_builder->CreatePointerCast(
        base_address,
        llvm::PointerType::get(
            llvm_visitor->get_llvm_type(t.fortran_array_base_element()),
            /* AddressSpace */ 0));

    return llvm_visitor->ir_builder->CreateGEP(base_address, { val_addr });
}

llvm::Value *FortranVisitorLLVMExpression::
    address_array_ith_element_via_pointer_arithmetic(
        TL::Type t,
        llvm::Value *base_address,
        const std::vector<llvm::Value *> indexes)
{
    int rank = t.fortran_rank();

    // Horner
    std::vector<llvm::Value *> offset_list, size_list;
    offset_list.reserve(indexes.size());
    size_list.reserve(indexes.size());

    for (int i = 0; i < rank; i++)
    {
        Nodecl::NodeclBase lower, upper;
        t.array_get_bounds(lower, upper);
        llvm::Value *val_lower = llvm_visitor->eval_expression(lower);
        llvm::Value *val_upper = llvm_visitor->eval_expression(upper);

        if (t.array_is_region())
        {
            Nodecl::NodeclBase region_lower, region_upper, region_stride;
            t.array_get_region_bounds(
                region_lower, region_upper, region_stride);

            llvm::Value *val_region_lower
                = llvm_visitor->eval_expression(region_lower);

            llvm::Value *offset = llvm_visitor->ir_builder->CreateAdd(
                llvm_visitor->ir_builder->CreateSub(val_region_lower,
                                                    val_lower),
                llvm_visitor->ir_builder->CreateMul(
                    indexes[i], llvm_visitor->eval_expression(region_stride)));

            offset_list.push_back(offset);
        }
        else
        {
            offset_list.push_back(indexes[i]);
        }

        llvm::Value *val_size = llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateSub(val_upper, val_lower),
            llvm_visitor->get_integer_value_64(1));
        size_list.push_back(val_size);

        t = t.array_element();
    }

    std::vector<llvm::Value *>::iterator it_offsets = offset_list.begin();
    std::vector<llvm::Value *>::iterator it_sizes = size_list.begin();

    llvm::Value *val_addr = *it_offsets;
    it_offsets++;
    it_sizes++;

    while (it_offsets != offset_list.end() && it_sizes != size_list.end())
    {
        val_addr = llvm_visitor->ir_builder->CreateAdd(
            *it_offsets,
            llvm_visitor->ir_builder->CreateMul(*it_sizes, val_addr));

        it_offsets++;
        it_sizes++;
    }

    ERROR_CONDITION(it_offsets != offset_list.end()
                        || it_sizes != size_list.end(),
                    "Lists do not match",
                    0);

    // Now multiply by the size of the type to get an offset in bytes
    ERROR_CONDITION(t.is_fortran_array(), "Should not be an array here", 0);

    return llvm_visitor->ir_builder->CreateGEP(base_address, { val_addr });
}

llvm::Value *FortranVisitorLLVMExpression::address_array_ith_element(
    const Nodecl::NodeclBase &array,
    TL::Type t,
    llvm::Value *base_address,
    const std::vector<llvm::Value *> indexes)
{
    t = t.no_ref();
    if (t.is_pointer())
        t = t.points_to();
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type!\n", 0);
    // Sanity check
    int rank = t.fortran_rank();
    ERROR_CONDITION(
        rank != (int)indexes.size(), "Mismatch between indexes and rank!\n", 0);

    if (t.array_requires_descriptor())
        return address_array_ith_element_via_descriptor(
            array, t, base_address, indexes);
    else
        return address_array_ith_element_via_pointer_arithmetic(
            t, base_address, indexes);
}

std::vector<llvm::Value *> FortranVisitorLLVMExpression::derref_indexes(
    const std::vector<llvm::Value *> v)
{
    std::vector<llvm::Value *> ret(v.begin(), v.end());
    for (auto &v : ret)
    {
        v = llvm_visitor->ir_builder->CreateLoad(v);
    }
    return ret;
}


template <typename Creator>
void FortranVisitorLLVMExpression::array_assignment(
    const Nodecl::NodeclBase &lhs,
    const Nodecl::NodeclBase &rhs,
    TL::Type lhs_type,
    TL::Type rhs_type,
    llvm::Value *lhs_addr,
    llvm::Value *rhs_addr,
    Creator create_store)
{
    LoopInfoOp loop_info_op;
    create_loop_header_for_array_op(rhs, rhs_type, rhs_addr, loop_info_op);

    // Loop body
    std::vector<llvm::Value *> idx_val = derref_indexes(loop_info_op.idx_var);
    llvm::Value *lhs_addr_element
        = address_array_ith_element(lhs, lhs_type, lhs_addr, idx_val);
    llvm::Value *rhs_addr_element
        = address_array_ith_element(rhs, rhs_type, rhs_addr, idx_val);
    create_store(llvm_visitor->ir_builder->CreateLoad(rhs_addr_element),
                 lhs_addr_element);
    create_loop_footer_for_array_op(rhs_type, loop_info_op);
}

typedef std::function<void(llvm::Value *addr, llvm::Value *value)> AssigOp;
AssigOp FortranVisitorLLVMExpression::get_assig_op(TL::Type lhs_type,
                                                   TL::Type rhs_type)
{
    if (lhs_type.no_ref().is_fortran_character()
        && rhs_type.no_ref().is_fortran_character())
    {
        return
            [&, lhs_type, rhs_type, this](llvm::Value *rhs, llvm::Value *lhs) {
                // A CHARACTER assignment
                // Fortran is a bit special in that its strings are padded with
                // blanks so we will copy the characters from rhs and then fill
                // whatever remains with blanks

                // FIXME - What about substrings?

                // Compute the minimum length of characters to transfer from rhs
                // to lhs
                llvm::Value *size_lhs
                    = llvm_visitor->eval_length_of_character(lhs_type.no_ref());
                llvm::Value *size_rhs
                    = llvm_visitor->eval_length_of_character(rhs_type);

                llvm::Value *min_size = llvm_visitor->ir_builder->CreateSelect(
                    llvm_visitor->ir_builder->CreateICmpSLT(size_lhs, size_rhs),
                    size_lhs,
                    size_rhs);

                // FIXME: CHARACTER(LEN=*) :: C should be handled like a VLA
                llvm::Value *lhs_addr = llvm_visitor->ir_builder->CreateGEP(
                    lhs,
                    { llvm_visitor->get_integer_value_32(0),
                      llvm_visitor->get_integer_value_64(0) });
                llvm::Value *rhs_addr = rhs;

                llvm_visitor->ir_builder->CreateMemCpy(
                    lhs_addr, rhs_addr, min_size, 1);

                llvm::Value *diff
                    = llvm_visitor->ir_builder->CreateSub(size_lhs, size_rhs);
                diff = llvm_visitor->ir_builder->CreateSelect(
                    llvm_visitor->ir_builder->CreateICmpSLT(
                        diff, llvm_visitor->get_integer_value_64(0)),
                    llvm_visitor->get_integer_value_64(0),
                    diff);

                llvm::Value *lhs_offset = llvm_visitor->ir_builder->CreateGEP(
                    lhs, { llvm_visitor->get_integer_value_64(0), min_size });

                llvm::Value *pad_len
                    = llvm_visitor->ir_builder->CreateSub(size_lhs, min_size);

                llvm_visitor->ir_builder->CreateMemSet(
                    lhs_offset,
                    llvm_visitor->get_integer_value_N(
                        ' ', llvm_visitor->llvm_types.i8, 8),
                    pad_len,
                    1);
            };
    }
    else
    {
        // A simple scalar assignment
        return [&, this](llvm::Value *rhs, llvm::Value *lhs) {
            llvm_visitor->ir_builder->CreateStore(rhs, lhs);
        };
    }
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Assignment &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    llvm::Value *vlhs = llvm_visitor->eval_expression(node.get_lhs());
    llvm::Value *vrhs = llvm_visitor->eval_expression(node.get_rhs());

    TL::Type lhs_type = node.get_lhs().get_type();
    TL::Type rhs_type = node.get_rhs().get_type();

    if (lhs_type.no_ref().is_fortran_array() && rhs_type.is_fortran_array())
    {
        array_assignment(node.get_lhs(),
                         node.get_rhs(),
                         lhs_type,
                         rhs_type,
                         vlhs,
                         vrhs,
                         get_assig_op(lhs_type, rhs_type));
    }
    else if (!lhs_type.no_ref().is_fortran_array()
             && !rhs_type.is_fortran_array())
    {
        auto create_store = get_assig_op(lhs_type, rhs_type);
        create_store(vrhs, vlhs);
    }
    else
    {
        internal_error("Unexpected assignment with lhs=%s and rhs=%s\n",
                       print_declarator(lhs_type.get_internal_type()),
                       print_declarator(rhs_type.get_internal_type()));
    }
    value = vlhs;
}


// void visit(const Nodecl::StructuredValue& node);
void FortranVisitorLLVMExpression::visit(const Nodecl::BooleanLiteral &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    const_value_t *val = node.get_constant();
    if (const_value_is_zero(val))
    {
        value = llvm_visitor->get_integer_value(0, node.get_type());
    }
    else
    {
        value = llvm_visitor->get_integer_value(1, node.get_type());
    }
}

void FortranVisitorLLVMExpression::visit(const Nodecl::ComplexLiteral &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    TL::Type base_type = node.get_type().complex_get_base_type();
    const_value_t *cval = node.get_constant();

    llvm::Value *real_value, *imag_value;
    if (base_type.is_float())
    {
        real_value = value = llvm::ConstantFP::get(
            llvm_visitor->llvm_context,
            llvm::APFloat(const_value_cast_to_float(
                const_value_complex_get_real_part(cval))));
        imag_value = value = llvm::ConstantFP::get(
            llvm_visitor->llvm_context,
            llvm::APFloat(const_value_cast_to_float(
                const_value_complex_get_imag_part(cval))));
    }
    else if (base_type.is_double())
    {
        real_value = value = llvm::ConstantFP::get(
            llvm_visitor->llvm_context,
            llvm::APFloat(const_value_cast_to_double(
                const_value_complex_get_real_part(cval))));
        imag_value = value = llvm::ConstantFP::get(
            llvm_visitor->llvm_context,
            llvm::APFloat(const_value_cast_to_double(
                const_value_complex_get_imag_part(cval))));
    }
    else
    {
        internal_error("Unexpected type '%s'",
                       print_declarator(base_type.get_internal_type()));
    }

    value
        = llvm::ConstantVector::get({ llvm::cast<llvm::Constant>(real_value),
                                      llvm::cast<llvm::Constant>(imag_value) });
}

void FortranVisitorLLVMExpression::visit(const Nodecl::ObjectInit &node)
{
    std::cerr << "Unhandled object init. Symbol = "
              << node.get_symbol().get_name() << " "
              << symbol_kind_name(node.get_symbol().get_internal_symbol())
              << std::endl;
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Neg &node)
{
    Nodecl::NodeclBase rhs = node.get_rhs();
    TL::Type rhs_type = rhs.get_type().no_ref();
    if (rhs_type.is_fortran_array())
        rhs_type = rhs_type.fortran_array_base_element();

    // There is no neg instruction. So "neg x" is represented as "sub 0, x"
    if (rhs_type.is_signed_integral())
    {
        unary_operator(
            node, [&, this](const Nodecl::NodeclBase &, llvm::Value *vrhs) {
                llvm::Value *z = llvm_visitor->get_integer_value(0, rhs_type);
                return llvm_visitor->ir_builder->CreateSub(z, vrhs);
            });
    }
    else if (rhs_type.is_floating_type())
    {
        unary_operator(node,
                       [&, this](const Nodecl::NodeclBase &,
                                 llvm::Value *vrhs) -> llvm::Value * {
                           llvm::Value *z = llvm::ConstantFP::get(
                               llvm_visitor->llvm_context, llvm::APFloat(0.0));
                           return llvm_visitor->ir_builder->CreateFSub(z, vrhs);
                       });
    }
    else
    {
        internal_error("Code unreachable for unary operator %s. Type is '%s'",
                       ast_print_node_type(node.get_kind()),
                       print_declarator(rhs.get_type().get_internal_type()));
    }
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Plus &node)
{
    // No-operation
    walk(node.get_rhs());
}

void FortranVisitorLLVMExpression::visit(const Nodecl::LogicalAnd &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    auto non_strict_logical_and = [&, this](
        const Nodecl::NodeclBase &node,
        const Nodecl::NodeclBase &lhs,
        const Nodecl::NodeclBase &rhs,
        std::function<llvm::Value *(const Nodecl::NodeclBase &)> eval_lhs,
        std::function<llvm::Value *(const Nodecl::NodeclBase &)> eval_rhs) {

        llvm::Value *vlhs = eval_lhs(lhs);

        llvm::BasicBlock *block_eval_lhs = llvm_visitor->get_current_block();

        llvm::Value *vcond = llvm_visitor->ir_builder->CreateZExtOrTrunc(
            vlhs, llvm_visitor->llvm_types.i1);

        llvm::BasicBlock *block_eval_rhs
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "and.rhs",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *block_end
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "and.end",
                                       llvm_visitor->get_current_function());
        llvm_visitor->ir_builder->CreateCondBr(
            vcond, block_eval_rhs, block_end);

        llvm_visitor->set_current_block(block_eval_rhs);
        llvm::Value *vrhs = eval_rhs(rhs);

        llvm_visitor->ir_builder->CreateBr(block_end);

        llvm_visitor->set_current_block(block_end);
        llvm::Value *result = llvm_visitor->ir_builder->CreatePHI(
            llvm_visitor->get_llvm_type(node.get_type()), 2);
        llvm::cast<llvm::PHINode>(result)->addIncoming(vlhs, block_eval_lhs);
        llvm::cast<llvm::PHINode>(result)->addIncoming(vrhs, block_eval_rhs);

        return result;
    };

    return binary_operator_non_strict(node, non_strict_logical_and);
}

void FortranVisitorLLVMExpression::visit(const Nodecl::LogicalOr &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    auto non_strict_logical_or = [&, this](
        const Nodecl::NodeclBase &node,
        const Nodecl::NodeclBase &lhs,
        const Nodecl::NodeclBase &rhs,
        std::function<llvm::Value *(const Nodecl::NodeclBase &)> eval_lhs,
        std::function<llvm::Value *(const Nodecl::NodeclBase &)> eval_rhs) {
        llvm::BasicBlock *block_eval_lhs = llvm_visitor->get_current_block();
        llvm::Value *vlhs = eval_lhs(lhs);

        llvm::Value *vcond = llvm_visitor->ir_builder->CreateZExtOrTrunc(
            vlhs, llvm_visitor->llvm_types.i1);

        llvm::BasicBlock *block_eval_rhs
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "or.rhs",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *block_end
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "or.end",
                                       llvm_visitor->get_current_function());
        llvm_visitor->ir_builder->CreateCondBr(
            vcond, block_end, block_eval_rhs);

        llvm_visitor->set_current_block(block_eval_rhs);
        llvm::Value *vrhs = eval_rhs(rhs);

        llvm_visitor->ir_builder->CreateBr(block_end);

        llvm_visitor->set_current_block(block_end);
        llvm::Value *result = llvm_visitor->ir_builder->CreatePHI(
            llvm_visitor->get_llvm_type(node.get_type()), 2);
        llvm::cast<llvm::PHINode>(result)->addIncoming(vlhs, block_eval_lhs);
        llvm::cast<llvm::PHINode>(result)->addIncoming(vrhs, block_eval_rhs);

        return result;
    };

    return binary_operator_non_strict(node, non_strict_logical_or);
}

void FortranVisitorLLVMExpression::visit(const Nodecl::LogicalNot &node)
{
    Nodecl::NodeclBase rhs = node.get_rhs();
    TL::Type rhs_type = rhs.get_type().no_ref();
    if (rhs_type.is_fortran_array())
        rhs_type = rhs_type.fortran_array_base_element();

    return unary_operator(
        node, [&, this](const Nodecl::NodeclBase, llvm::Value *vrhs) {
            // select x, 0, 1
            return llvm_visitor->ir_builder->CreateZExtOrTrunc(
                llvm_visitor->ir_builder->CreateSelect(
                    llvm_visitor->ir_builder->CreateZExtOrTrunc(
                        vrhs, llvm_visitor->llvm_types.i1),
                    llvm_visitor->get_integer_value(0, rhs_type),
                    llvm_visitor->get_integer_value(1, rhs_type)),
                llvm_visitor->get_llvm_type(node.get_type()));
        });
}

template <typename Create>
llvm::Value *FortranVisitorLLVMExpression::binary_operator_scalar(
    Nodecl::NodeclBase lhs,
    Nodecl::NodeclBase rhs,
    llvm::Value *lhs_val,
    llvm::Value *rhs_val,
    Create create)
{
    return create(lhs, rhs, lhs_val, rhs_val);
}

llvm::Value *FortranVisitorLLVMExpression::compute_offset_from_linear_element(
    TL::Type t, llvm::Value *idx_value)
{
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type", 0);

    TL::Type element_type = t.array_element();
    llvm::Value *element_size_bytes
        = llvm_visitor->eval_sizeof_64(element_type);

    if (t.array_requires_descriptor())
    {
        internal_error("Not yet implemented", 0);
    }
    else if (t.array_is_region())
    {
        // The idea is that eval_elements_of_dimension has taken into
        // account the array subscript, so given an array subscript
        // A(L:U:S) and an index I the address we want to compute is L + I
        // * S, if S > 0, or U + I * S, if S < 0.
        Nodecl::NodeclBase region_lower
            = array_type_get_region_lower_bound(t.get_internal_type());
        Nodecl::NodeclBase region_upper
            = array_type_get_region_upper_bound(t.get_internal_type());
        Nodecl::NodeclBase stride
            = array_type_get_region_stride(t.get_internal_type());

        // We try to be a bit smart here
        llvm::Value *offset = nullptr;
        if (stride.is_constant())
        {
            const_value_t *cval_stride = stride.get_constant();
            if (const_value_is_positive(cval_stride))
            {
                llvm::Value *vregion_lower
                    = llvm_visitor->eval_expression(region_lower);
                if (const_value_is_one(cval_stride))
                {
                    // Simplest case (L + I)
                    offset = llvm_visitor->ir_builder->CreateAdd(vregion_lower,
                                                                 idx_value);
                }
                else
                {
                    // (L + I * S)
                    llvm::Value *vstride
                        = llvm_visitor->eval_expression(stride);
                    offset = llvm_visitor->ir_builder->CreateAdd(
                        vregion_lower,
                        llvm_visitor->ir_builder->CreateMul(idx_value,
                                                            vstride));
                }
            }
            else if (const_value_is_negative(cval_stride))
            {
                // (U + I * S), S < 0
                llvm::Value *vregion_upper
                    = llvm_visitor->eval_expression(region_upper);
                llvm::Value *vstride = llvm_visitor->eval_expression(stride);
                offset = llvm_visitor->ir_builder->CreateAdd(
                    vregion_upper,
                    llvm_visitor->ir_builder->CreateMul(idx_value, vstride));
            }
            else
            {
                internal_error("Code unreachable", 0);
            }

            // Offset with the lower element of the array (which may not be
            // the same as the region)
            Nodecl::NodeclBase array_lower
                = array_type_get_array_lower_bound(t.get_internal_type());
            llvm::Value *varray_lower
                = llvm_visitor->eval_expression(array_lower);
            offset = llvm_visitor->ir_builder->CreateSub(offset, varray_lower);

            // Multiply with element size
            offset = llvm_visitor->ir_builder->CreateMul(element_size_bytes,
                                                         offset);
            return offset;
        }
        else
        {
            internal_error("Not implemented yet", 0);
        }
    }
    else
    {
        // Contiguous array
        llvm::Value *offset = llvm_visitor->ir_builder->CreateMul(
            element_size_bytes, idx_value);

        return offset;
    }
}


template <typename Create>
void FortranVisitorLLVMExpression::binary_operator_array(
    const Nodecl::NodeclBase &node,
    const Nodecl::NodeclBase &lhs,
    const Nodecl::NodeclBase &rhs,
    Create create)
{
    TL::Type node_type = node.get_type();

    ERROR_CONDITION(
        !node_type.is_fortran_array(), "The result must be an array!\n", 0);

    TL::Type node_element_type = node_type.array_base_element();

    llvm::Value *array_size = llvm_visitor->eval_elements_of_array(
        node, node_type, /* addr */ nullptr);

    // Allocate space for the result
    llvm::Value *result_addr = llvm_visitor->ir_builder->CreateAlloca(
        llvm_visitor->get_llvm_type(node_element_type), array_size);

    // Index inside the contiguous result array. This is used for looping.
    llvm::Value *result_idx_addr
        = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
    llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0),
                                          result_idx_addr);

    LoopInfoOp loop_info_op;
    create_loop_header_for_array_op(
        node, node_type, /* addr */ nullptr, loop_info_op);
    std::vector<llvm::Value *> idx_val = derref_indexes(loop_info_op.idx_var);

    TL::Type lhs_type = lhs.get_type();
    llvm::Value *lhs_value;
    if (lhs_type.is_fortran_array())
    {
        llvm::Value *lhs_addr = llvm_visitor->eval_expression(lhs);
        llvm::Value *lhs_addr_element
            = address_array_ith_element(lhs, lhs_type, lhs_addr, idx_val);
        lhs_value = llvm_visitor->ir_builder->CreateLoad(lhs_addr_element);
    }
    else
    {
        lhs_value = llvm_visitor->eval_expression(lhs);
    }

    TL::Type rhs_type = rhs.get_type();
    llvm::Value *rhs_value;
    if (rhs_type.is_fortran_array())
    {
        llvm::Value *rhs_addr = llvm_visitor->eval_expression(rhs);
        llvm::Value *rhs_addr_element
            = address_array_ith_element(rhs, rhs_type, rhs_addr, idx_val);
        rhs_value = llvm_visitor->ir_builder->CreateLoad(rhs_addr_element);
    }
    else
    {
        rhs_value = llvm_visitor->eval_expression(rhs);
    }

    llvm::Value *val_op = create(lhs, rhs, lhs_value, rhs_value);

    // FIXME - CHARACTER assignment!
    llvm_visitor->ir_builder->CreateStore(
        val_op,
        llvm_visitor->ir_builder->CreateGEP(
            result_addr,
            { llvm_visitor->ir_builder->CreateLoad(result_idx_addr) }));

    llvm_visitor->ir_builder->CreateStore(
        llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateLoad(result_idx_addr),
            llvm_visitor->get_integer_value_64(1)),
        result_idx_addr);
    create_loop_footer_for_array_op(node_type, loop_info_op);

    // The result array is an address in LLVM world.
    value = result_addr;
}

template <typename Node, typename Create>
void FortranVisitorLLVMExpression::binary_operator(const Node node,
                                                   Create create)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    Nodecl::NodeclBase lhs = node.get_lhs();
    Nodecl::NodeclBase rhs = node.get_rhs();

    TL::Type lhs_type = lhs.get_type();
    TL::Type rhs_type = rhs.get_type();

    if (lhs_type.is_fortran_array() || rhs_type.is_fortran_array())
    {
        return binary_operator_array(node, lhs, rhs, create);
    }
    else
    {
        // A scalar intrinsic binary operation
        llvm::Value *vlhs = llvm_visitor->eval_expression(lhs);
        llvm::Value *vrhs = llvm_visitor->eval_expression(rhs);

        value = binary_operator_scalar(lhs, rhs, vlhs, vrhs, create);
    }
}

template <typename NonStrictOpt>
void FortranVisitorLLVMExpression::binary_operator_array_non_strict(
    const Nodecl::NodeclBase &node,
    const Nodecl::NodeclBase &lhs,
    const Nodecl::NodeclBase &rhs,
    NonStrictOpt non_strict_op)
{
    TL::Type node_type = node.get_type();

    ERROR_CONDITION(
        !node_type.is_fortran_array(), "The result must be an array!\n", 0);

    TL::Type node_element_type = node_type.array_base_element();

    llvm::Value *array_size = llvm_visitor->eval_elements_of_array(
        node, node_type, /* addr */ nullptr);

    // Allocate space for the result
    llvm::Value *result_addr = llvm_visitor->ir_builder->CreateAlloca(
        llvm_visitor->get_llvm_type(node_element_type), array_size);

    // Index inside the contiguous result array. This is used for looping.
    llvm::Value *result_idx_addr
        = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
    llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0),
                                          result_idx_addr);

    LoopInfoOp loop_info_op;
    create_loop_header_for_array_op(
        node, node_type, /* addr */ nullptr, loop_info_op);
    std::vector<llvm::Value *> idx_val = derref_indexes(loop_info_op.idx_var);

    auto eval_lhs = [&, this](Nodecl::NodeclBase lhs) -> llvm::Value * {
        TL::Type lhs_type = lhs.get_type();
        llvm::Value *lhs_value;
        if (lhs_type.is_fortran_array())
        {
            llvm::Value *lhs_addr = llvm_visitor->eval_expression(lhs);
            llvm::Value *lhs_addr_element
                = address_array_ith_element(lhs, lhs_type, lhs_addr, idx_val);
            lhs_value = llvm_visitor->ir_builder->CreateLoad(lhs_addr_element);
        }
        else
        {
            lhs_value = llvm_visitor->eval_expression(lhs);
        }
        return lhs_value;
    };

    auto eval_rhs = [&, this](Nodecl::NodeclBase rhs) -> llvm::Value * {
        TL::Type rhs_type = rhs.get_type();
        llvm::Value *rhs_value;
        if (rhs_type.is_fortran_array())
        {
            llvm::Value *rhs_addr = llvm_visitor->eval_expression(rhs);
            llvm::Value *rhs_addr_element
                = address_array_ith_element(rhs, rhs_type, rhs_addr, idx_val);
            rhs_value = llvm_visitor->ir_builder->CreateLoad(rhs_addr_element);
        }
        else
        {
            rhs_value = llvm_visitor->eval_expression(rhs);
        }
        return rhs_value;
    };

    llvm::Value *val_op = non_strict_op(node, lhs, rhs, eval_lhs, eval_rhs);

    // FIXME - CHARACTER assignment!
    llvm_visitor->ir_builder->CreateStore(
        val_op,
        llvm_visitor->ir_builder->CreateGEP(
            result_addr,
            { llvm_visitor->ir_builder->CreateLoad(result_idx_addr) }));

    llvm_visitor->ir_builder->CreateStore(
        llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateLoad(result_idx_addr),
            llvm_visitor->get_integer_value_64(1)),
        result_idx_addr);
    create_loop_footer_for_array_op(node_type, loop_info_op);

    // The result array is an address in LLVM world.
    value = result_addr;
}

template <typename Node, typename NonStrictOp>
void FortranVisitorLLVMExpression::binary_operator_non_strict(
    const Node node, NonStrictOp non_strict_op)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    Nodecl::NodeclBase lhs = node.get_lhs();
    Nodecl::NodeclBase rhs = node.get_rhs();

    TL::Type lhs_type = lhs.get_type();
    TL::Type rhs_type = rhs.get_type();

    if (lhs_type.is_fortran_array() || rhs_type.is_fortran_array())
    {
        return binary_operator_array_non_strict(
            node, node.get_lhs(), node.get_rhs(), non_strict_op);
    }
    else
    {
        non_strict_op(node,
                      node.get_lhs(),
                      node.get_rhs(),
                      [&, this](Nodecl::NodeclBase lhs) -> llvm::Value * {
                          return llvm_visitor->eval_expression(lhs);
                      },
                      [&, this](Nodecl::NodeclBase rhs) -> llvm::Value * {
                          return llvm_visitor->eval_expression(rhs);
                      });
    }
}

template <typename Create>
llvm::Value *FortranVisitorLLVMExpression::unary_operator_scalar(
    Nodecl::NodeclBase rhs, llvm::Value *rhs_val, Create create)
{
    return create(rhs, rhs_val);
}

template <typename Create>
void FortranVisitorLLVMExpression::unary_operator_array(
    const Nodecl::NodeclBase &node,
    const Nodecl::NodeclBase &rhs,
    Create create)
{
    TL::Type node_type = node.get_type();

    ERROR_CONDITION(
        !node_type.is_fortran_array(), "The result must be an array!\n", 0);

    TL::Type node_element_type = node_type.array_base_element();

    llvm::Value *array_size = llvm_visitor->eval_elements_of_array(
        node, node_type, /* addr */ nullptr);

    // Allocate space for the result
    llvm::Value *result_addr = llvm_visitor->ir_builder->CreateAlloca(
        llvm_visitor->get_llvm_type(node_element_type), array_size);

    // Index inside the contiguous result array. This is used for looping.
    llvm::Value *result_idx_addr
        = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
    llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0),
                                          result_idx_addr);

    LoopInfoOp loop_info_op;
    create_loop_header_for_array_op(
        node, node_type, /* addr */ nullptr, loop_info_op);
    std::vector<llvm::Value *> idx_val = derref_indexes(loop_info_op.idx_var);

    TL::Type rhs_type = rhs.get_type();
    llvm::Value *rhs_addr = llvm_visitor->eval_expression(rhs);
    llvm::Value *rhs_addr_element
        = address_array_ith_element(rhs, rhs_type, rhs_addr, idx_val);
    llvm::Value *rhs_value
        = llvm_visitor->ir_builder->CreateLoad(rhs_addr_element);

    llvm::Value *val_op = create(rhs, rhs_value);

    // FIXME - CHARACTER assignment!
    llvm_visitor->ir_builder->CreateStore(
        val_op,
        llvm_visitor->ir_builder->CreateGEP(
            result_addr,
            { llvm_visitor->ir_builder->CreateLoad(result_idx_addr) }));

    llvm_visitor->ir_builder->CreateStore(
        llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateLoad(result_idx_addr),
            llvm_visitor->get_integer_value_64(1)),
        result_idx_addr);
    create_loop_footer_for_array_op(node_type, loop_info_op);

    // The result array is an address in LLVM world.
    value = result_addr;
}

template <typename Node, typename Create>
void FortranVisitorLLVMExpression::unary_operator(const Node node,
                                                  Create create)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    Nodecl::NodeclBase rhs = node.get_rhs();
    TL::Type rhs_type = rhs.get_type();

    if (rhs_type.is_fortran_array())
    {
        return unary_operator_array(node, rhs, create);
    }
    else
    {
        // A scalar intrinsic binary operation
        llvm::Value *vrhs = llvm_visitor->eval_expression(rhs);
        value = unary_operator_scalar(rhs, vrhs, create);
    }
}

llvm::Value *FortranVisitorLLVMExpression::create_complex_value(
    TL::Type complex_type, llvm::Value *real, llvm::Value *imag)
{
    llvm::Value *result
        = llvm::UndefValue::get(llvm_visitor->get_llvm_type(complex_type));
    result = llvm_visitor->ir_builder->CreateInsertElement(
        result, real, uint64_t(0));
    result = llvm_visitor->ir_builder->CreateInsertElement(result, imag, 1);
    return result;
}

#define CREATOR(Creator)                                    \
    [&, this](Nodecl::NodeclBase,                           \
              Nodecl::NodeclBase,                           \
              llvm::Value *lhs,                             \
              llvm::Value *rhs) {                           \
        return llvm_visitor->ir_builder->Creator(lhs, rhs); \
    }
#define CREATOR_CMP(Creator, node)                           \
    [&, this](Nodecl::NodeclBase,                            \
              Nodecl::NodeclBase,                            \
              llvm::Value *lhs,                              \
              llvm::Value *rhs) {                            \
        return llvm_visitor->ir_builder->CreateZExtOrTrunc(  \
            llvm_visitor->ir_builder->Creator(lhs, rhs),     \
            llvm_visitor->get_llvm_type((node).get_type())); \
    }
#define INVALID_OP                                 \
    [&, this](Nodecl::NodeclBase,                  \
              Nodecl::NodeclBase,                  \
              llvm::Value *lhs,                    \
              llvm::Value *rhs) -> llvm::Value * { \
        internal_error("Invalid operation", 0);    \
        return nullptr;                            \
    }
#define UNIMPLEMENTED_OP                                    \
    [&, this](Nodecl::NodeclBase,                           \
              Nodecl::NodeclBase,                           \
              llvm::Value *lhs,                             \
              llvm::Value *rhs) -> llvm::Value * {          \
        internal_error("Operation not yet implemented", 0); \
        return nullptr;                                     \
    }

typedef std::function<llvm::Value *(
    Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value *, llvm::Value *)>
    BinaryOpCreator;
BinaryOpCreator FortranVisitorLLVMExpression::choose_arithmetic_creator(
    TL::Type t,
    BinaryOpCreator create_integer,
    BinaryOpCreator create_real,
    BinaryOpCreator create_complex)
{
    t = t.no_ref();
    if (t.is_fortran_array())
        t = t.fortran_array_base_element();
    if (t.is_signed_integral())
        return create_integer;
    else if (t.is_floating_type())
        return create_real;
    else if (t.is_complex())
        return create_complex;
    else
        internal_error("Unexpected type '%s' for arithmetic binary operator\n",
                       print_declarator(t.get_internal_type()));
}

template <typename Node>
void FortranVisitorLLVMExpression::arithmetic_binary_operator(
    Node node,
    BinaryOpCreator create_integer,
    BinaryOpCreator create_real,
    BinaryOpCreator create_complex)
{
    binary_operator(
        node,
        choose_arithmetic_creator(
            node.get_type(), create_integer, create_real, create_complex));
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Add &node)
{
    arithmetic_binary_operator(
        node, CREATOR(CreateAdd), CREATOR(CreateFAdd), CREATOR(CreateFAdd));
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Minus &node)
{
    arithmetic_binary_operator(
        node, CREATOR(CreateSub), CREATOR(CreateFSub), CREATOR(CreateFSub));
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Mul &node)
{
    auto create_complex_mul = [&, this](Nodecl::NodeclBase,
                                        Nodecl::NodeclBase,
                                        llvm::Value *lhs,
                                        llvm::Value *rhs) {
        // (a, b) * (c, d) = (ac - bd, ad + bc)
        llvm::Value *a
            = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(0));
        llvm::Value *b = llvm_visitor->ir_builder->CreateExtractElement(lhs, 1);
        llvm::Value *c
            = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(0));
        llvm::Value *d = llvm_visitor->ir_builder->CreateExtractElement(rhs, 1);

        llvm::Value *real_part = llvm_visitor->ir_builder->CreateFSub(
            llvm_visitor->ir_builder->CreateFMul(a, c),
            llvm_visitor->ir_builder->CreateFMul(b, d));
        llvm::Value *imag_part = llvm_visitor->ir_builder->CreateFAdd(
            llvm_visitor->ir_builder->CreateFMul(a, d),
            llvm_visitor->ir_builder->CreateFMul(b, c));

        return create_complex_value(node.get_type(), real_part, imag_part);
    };

    arithmetic_binary_operator(
        node, CREATOR(CreateMul), CREATOR(CreateFMul), create_complex_mul);
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Div &node)
{
    auto create_complex_div = [&, this](Nodecl::NodeclBase,
                                        Nodecl::NodeclBase,
                                        llvm::Value *lhs,
                                        llvm::Value *rhs) {
        // (a, b) / (c, d) = ((ac + bd) / (c^2 + d^2), (bc - ad) / (c^2 + d^2))
        llvm::Value *a
            = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(0));
        llvm::Value *b = llvm_visitor->ir_builder->CreateExtractElement(lhs, 1);
        llvm::Value *c
            = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(0));
        llvm::Value *d = llvm_visitor->ir_builder->CreateExtractElement(rhs, 1);

        llvm::Value *divisor = llvm_visitor->ir_builder->CreateFAdd(
            llvm_visitor->ir_builder->CreateFMul(c, c),
            llvm_visitor->ir_builder->CreateFMul(d, d));

        llvm::Value *real_part = llvm_visitor->ir_builder->CreateFDiv(
            llvm_visitor->ir_builder->CreateFAdd(
                llvm_visitor->ir_builder->CreateFMul(a, c),
                llvm_visitor->ir_builder->CreateFMul(b, d)),
            divisor);
        llvm::Value *imag_part = llvm_visitor->ir_builder->CreateFDiv(
            llvm_visitor->ir_builder->CreateFSub(
                llvm_visitor->ir_builder->CreateFMul(b, c),
                llvm_visitor->ir_builder->CreateFMul(a, d)),
            divisor);

        return create_complex_value(node.get_type(), real_part, imag_part);
    };
    arithmetic_binary_operator(
        node, CREATOR(CreateSDiv), CREATOR(CreateFDiv), create_complex_div);
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Power &node)
{
    // We use exponentiation by squaring
    // a ** b is either   a ** (2 * b) == (a*a) ** b
    //           or       a ** (2 * b + 1) == a * (a*a) ** b
    auto create_pow_int = [&, this](Nodecl::NodeclBase,
                                    Nodecl::NodeclBase,
                                    llvm::Value *lhs,
                                    llvm::Value *rhs) {
        llvm::BasicBlock *pow_loop_check
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "pow.loop.check",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *pow_loop_body
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "pow.loop.body",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *pow_loop_body_odd
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "pow.loop.body.odd",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *pow_loop_body_common
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "pow.loop.body.common",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *pow_loop_end
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "pow.loop.end",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *pow_if_neg
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "pow.if.neg",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *pow_if_end
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "pow.if.end",
                                       llvm_visitor->get_current_function());

        llvm::Value *result = llvm_visitor->ir_builder->CreateAlloca(
            llvm_visitor->get_llvm_type(node.get_type()));
        llvm::Value *base = llvm_visitor->ir_builder->CreateAlloca(
            llvm_visitor->get_llvm_type(node.get_type()));
        llvm::Value *exponent = llvm_visitor->ir_builder->CreateAlloca(
            llvm_visitor->get_llvm_type(node.get_type()));

        // rhs < 0
        llvm::Value *negative_exponent
            = llvm_visitor->ir_builder->CreateICmpSLT(
                rhs, llvm_visitor->get_integer_value(0, node.get_type()));

        // result <- 1
        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->get_integer_value(1, node.get_type()), result);
        // base <- lhs
        llvm_visitor->ir_builder->CreateStore(lhs, base);
        // exponent <- |rhs|
        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->ir_builder->CreateSelect(
                negative_exponent,
                llvm_visitor->ir_builder->CreateSub(
                    llvm_visitor->get_integer_value(0, node.get_type()), rhs),
                rhs),
            exponent);

        llvm_visitor->ir_builder->CreateBr(pow_loop_check);

        llvm_visitor->set_current_block(pow_loop_check);
        // exponent != 0
        llvm::Value *cond_val = llvm_visitor->ir_builder->CreateICmpNE(
            llvm_visitor->ir_builder->CreateLoad(exponent),
            llvm_visitor->get_integer_value(0, node.get_type()));
        llvm_visitor->ir_builder->CreateCondBr(
            cond_val, pow_loop_body, pow_loop_end);

        llvm_visitor->set_current_block(pow_loop_body);
        // exponent % 2
        cond_val = llvm_visitor->ir_builder->CreateZExtOrTrunc(
            llvm_visitor->ir_builder->CreateSRem(
                llvm_visitor->ir_builder->CreateLoad(exponent),
                llvm_visitor->get_integer_value(2, node.get_type())),
            llvm_visitor->llvm_types.i1);
        llvm_visitor->ir_builder->CreateCondBr(
            cond_val, pow_loop_body_odd, pow_loop_body_common);

        // exponent % 2 == 1
        llvm_visitor->set_current_block(pow_loop_body_odd);
        // result <- result * base
        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->ir_builder->CreateMul(
                llvm_visitor->ir_builder->CreateLoad(result),
                llvm_visitor->ir_builder->CreateLoad(base)),
            result);
        llvm_visitor->ir_builder->CreateBr(pow_loop_body_common);

        // Common case
        llvm_visitor->set_current_block(pow_loop_body_common);
        // exponent <- exponent / 2
        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->ir_builder->CreateSDiv(
                llvm_visitor->ir_builder->CreateLoad(exponent),
                llvm_visitor->get_integer_value(2, node.get_type())),
            exponent);
        // base <- base * base
        llvm::Value *base_load = llvm_visitor->ir_builder->CreateLoad(base);
        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->ir_builder->CreateMul(base_load, base_load), base);

        llvm_visitor->ir_builder->CreateBr(pow_loop_check);

        llvm_visitor->set_current_block(pow_loop_end);
        llvm_visitor->ir_builder->CreateCondBr(
            negative_exponent, pow_if_neg, pow_if_end);

        // If exponent was negative A ** (-B) is the same as 1 / (A ** B)
        llvm_visitor->set_current_block(pow_if_neg);
        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->ir_builder->CreateSDiv(
                llvm_visitor->get_integer_value(1, node.get_type()),
                llvm_visitor->ir_builder->CreateLoad(result)),
            result);
        llvm_visitor->ir_builder->CreateBr(pow_if_end);

        llvm_visitor->set_current_block(pow_if_end);
        return llvm_visitor->ir_builder->CreateLoad(result);
    };

    auto create_pow_float = [&, this](Nodecl::NodeclBase,
                                      Nodecl::NodeclBase,
                                      llvm::Value *lhs,
                                      llvm::Value *rhs) {
        llvm::Function *powf = llvm::Intrinsic::getDeclaration(
            llvm_visitor->current_module.get(),
            llvm::Intrinsic::pow,
            { lhs->getType(), rhs->getType() });
        ERROR_CONDITION(powf == nullptr, "llvm.pow not found?", 0);
        return llvm_visitor->ir_builder->CreateCall(powf, { lhs, rhs });
    };

    auto create_pow_complex = [&, this](Nodecl::NodeclBase,
                                        Nodecl::NodeclBase,
                                        llvm::Value *lhs,
                                        llvm::Value *rhs) {
        llvm::Type *complex_type = llvm_visitor->get_llvm_type(node.get_type());
        std::string cpow_name;
        TL::Type base_type = node.get_type().complex_get_base_type();
        if (base_type.is_float())
            cpow_name = "cpowf";
        else if (base_type.is_double())
            cpow_name = "cpow";
        else
            internal_error(
                "Unsupported type for complex power '%s'\n",
                print_declarator(node.get_type().get_internal_type()));

        llvm::Function *cpow_fun = llvm::cast<llvm::Function>(
            llvm_visitor->current_module->getOrInsertFunction(
                cpow_name,
                llvm::FunctionType::get(complex_type,
                                        { complex_type, complex_type },
                                        /* isVarArg */ false),
                /* no attributes so far */ llvm::AttributeSet()));
        return llvm_visitor->ir_builder->CreateCall(cpow_fun, { lhs, rhs });
    };

    arithmetic_binary_operator(
        node, create_pow_int, create_pow_float, create_pow_complex);
}

template <typename Node,
          typename CreateSInt,
          typename CreateFloat,
          typename CreateComplex,
          typename CreateCharacter>
void FortranVisitorLLVMExpression::binary_comparison(
    const Node node,
    CreateSInt create_sint,
    CreateFloat create_float,
    CreateComplex create_complex,
    CreateCharacter create_character)
{
    TL::Type lhs_type = node.get_lhs().get_type().no_ref();
    if (lhs_type.is_fortran_array())
        lhs_type = lhs_type.array_base_element();

    TL::Type rhs_type = node.get_rhs().get_type().no_ref();
    if (rhs_type.is_fortran_array())
        rhs_type = rhs_type.array_base_element();

    BinaryOpCreator create;
    if (lhs_type.is_signed_integral() && rhs_type.is_signed_integral())
        create = create_sint;
    else if (lhs_type.is_floating_type() && rhs_type.is_floating_type())
        create = create_float;
    else if (lhs_type.is_complex() && rhs_type.is_complex())
        create = create_complex;
    else if (lhs_type.is_fortran_character() && rhs_type.is_fortran_character())
        create = create_character;
    // This happens only for .EQV. and .NEQV.
    else if (lhs_type.is_bool() && rhs_type.is_bool())
        create = create_sint;
    else
        internal_error(
            "Unexpected type '%s' and '%s' for arithmetic binary relational "
            "operator\n",
            print_declarator(lhs_type.get_internal_type()),
            print_declarator(rhs_type.get_internal_type()));

    binary_operator(node, create);
}

FortranVisitorLLVMExpression::CharacterCompareLT::CharacterCompareLT(
    FortranLLVM *llvm_visitor, Nodecl::NodeclBase node)
    : llvm_visitor(llvm_visitor), node(node)
{
}

llvm::Value *FortranVisitorLLVMExpression::CharacterCompareLT::operator()(
    Nodecl::NodeclBase lhs,
    Nodecl::NodeclBase rhs,
    llvm::Value *vlhs,
    llvm::Value *vrhs)
{
    llvm::Value *len_lhs
        = llvm_visitor->eval_length_of_character(lhs.get_type());
    llvm::Value *len_rhs
        = llvm_visitor->eval_length_of_character(rhs.get_type());

    llvm::Value *len_min = llvm_visitor->ir_builder->CreateSelect(
        llvm_visitor->ir_builder->CreateICmpSLT(len_lhs, len_rhs),
        len_lhs,
        len_rhs);

    llvm::BasicBlock *loop_check
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.check",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_body
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.body",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_body_check
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.body.check",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_body_next
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.body.next",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_end
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.end",
                                   llvm_visitor->get_current_function());

    llvm::BasicBlock *tail_check
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.loop.check",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *tail_body
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.loop.body",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *tail_body_next
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.loop.body.next",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *tail_nonempty
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.nonempty",
                                   llvm_visitor->get_current_function());

    llvm::BasicBlock *cmp_true
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.true",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *cmp_false
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.false",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *cmp_end
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.end",
                                   llvm_visitor->get_current_function());

    // Normalize operands types
    vlhs = llvm_visitor->ir_builder->CreatePointerCast(
        vlhs, llvm_visitor->llvm_types.ptr_i8);
    vrhs = llvm_visitor->ir_builder->CreatePointerCast(
        vrhs, llvm_visitor->llvm_types.ptr_i8);

    // A first loop does the comparison between the prefix
    llvm::Value *idx_var
        = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
    llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0),
                                          idx_var);
    llvm_visitor->ir_builder->CreateBr(loop_check);

    llvm_visitor->set_current_block(loop_check);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), len_min),
        loop_body,
        loop_end);

    llvm_visitor->set_current_block(loop_body);

    llvm::Value *lhs_offset = llvm_visitor->ir_builder->CreateGEP(
        vlhs, { llvm_visitor->ir_builder->CreateLoad(idx_var) });
    llvm::Value *lhs_char = llvm_visitor->ir_builder->CreateLoad(lhs_offset);

    llvm::Value *rhs_offset = llvm_visitor->ir_builder->CreateGEP(
        vrhs, { llvm_visitor->ir_builder->CreateLoad(idx_var) });
    llvm::Value *rhs_char = llvm_visitor->ir_builder->CreateLoad(rhs_offset);

    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpEQ(lhs_char, rhs_char),
        loop_body_next,
        loop_body_check);

    llvm_visitor->set_current_block(loop_body_check);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLT(lhs_char, rhs_char),
        cmp_true,
        cmp_false);

    llvm_visitor->set_current_block(loop_body_next);
    llvm_visitor->ir_builder->CreateStore(
        llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateLoad(idx_var),
            llvm_visitor->get_integer_value_64(1)),
        idx_var);
    llvm_visitor->ir_builder->CreateBr(loop_check);

    llvm_visitor->set_current_block(loop_end);

    // A second loop makes sure that the remainder of the longest
    // string is all blanks
    llvm::Value *checked_character = llvm_visitor->ir_builder->CreateSelect(
        llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), len_lhs),
        vlhs,
        vrhs);
    llvm::Value *len_max = llvm_visitor->ir_builder->CreateSelect(
        llvm_visitor->ir_builder->CreateICmpSGT(len_lhs, len_rhs),
        len_lhs,
        len_rhs);
    llvm_visitor->ir_builder->CreateBr(tail_check);

    llvm_visitor->set_current_block(tail_check);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), len_max),
        tail_body,
        cmp_false);

    llvm_visitor->set_current_block(tail_body);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpEQ(
            llvm_visitor->ir_builder->CreateLoad(
                llvm_visitor->ir_builder->CreateGEP(
                    llvm_visitor->ir_builder->CreatePointerCast(
                        checked_character, llvm_visitor->llvm_types.ptr_i8),
                    { llvm_visitor->ir_builder->CreateLoad(idx_var) })),
            llvm_visitor->get_integer_value_N(
                ' ', llvm_visitor->llvm_types.i8, 8)),
        tail_body_next,
        tail_nonempty);

    llvm_visitor->set_current_block(tail_body_next);
    llvm_visitor->ir_builder->CreateStore(
        llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateLoad(idx_var),
            llvm_visitor->get_integer_value_64(1)),
        idx_var);
    llvm_visitor->ir_builder->CreateBr(tail_check);

    llvm_visitor->set_current_block(tail_nonempty);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLT(len_lhs, len_rhs),
        cmp_true,
        cmp_false);

    llvm_visitor->set_current_block(cmp_true);
    llvm::Value *result_true
        = llvm_visitor->get_integer_value_N(1, llvm_visitor->llvm_types.i1, 1);
    llvm_visitor->ir_builder->CreateBr(cmp_end);

    llvm_visitor->set_current_block(cmp_false);
    llvm::Value *result_false
        = llvm_visitor->get_integer_value_N(0, llvm_visitor->llvm_types.i1, 1);
    llvm_visitor->ir_builder->CreateBr(cmp_end);

    llvm_visitor->set_current_block(cmp_end);
    llvm::Value *value
        = llvm_visitor->ir_builder->CreatePHI(llvm_visitor->llvm_types.i1, 2);
    llvm::cast<llvm::PHINode>(value)->addIncoming(result_true, cmp_true);
    llvm::cast<llvm::PHINode>(value)->addIncoming(result_false, cmp_false);

    value = llvm_visitor->ir_builder->CreateZExtOrTrunc(
        value, llvm_visitor->get_llvm_type(node.get_type()));

    return value;
}

void FortranVisitorLLVMExpression::visit(const Nodecl::LowerThan &node)
{
    CharacterCompareLT character_compare_lt(llvm_visitor, node);
    binary_comparison(node,
                      CREATOR_CMP(CreateICmpSLT, node),
                      CREATOR_CMP(CreateFCmpOLT, node),
                      INVALID_OP,
                      character_compare_lt);
}

FortranVisitorLLVMExpression::CharacterCompareLE::CharacterCompareLE(
    FortranLLVM *llvm_visitor, Nodecl::NodeclBase node)
    : llvm_visitor(llvm_visitor), node(node)
{
}

llvm::Value *FortranVisitorLLVMExpression::CharacterCompareLE::operator()(
    Nodecl::NodeclBase lhs,
    Nodecl::NodeclBase rhs,
    llvm::Value *vlhs,
    llvm::Value *vrhs)
{
    llvm::Value *len_lhs
        = llvm_visitor->eval_length_of_character(lhs.get_type());
    llvm::Value *len_rhs
        = llvm_visitor->eval_length_of_character(rhs.get_type());

    llvm::Value *len_min = llvm_visitor->ir_builder->CreateSelect(
        llvm_visitor->ir_builder->CreateICmpSLT(len_lhs, len_rhs),
        len_lhs,
        len_rhs);

    llvm::BasicBlock *loop_check
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.check",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_body
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.body",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_body_check
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.body.check",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_body_next
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.body.next",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_end
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.end",
                                   llvm_visitor->get_current_function());

    llvm::BasicBlock *tail_check
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.loop.check",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *tail_body
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.loop.body",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *tail_body_next
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.loop.body.next",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *tail_nonempty
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.nonempty",
                                   llvm_visitor->get_current_function());

    llvm::BasicBlock *cmp_true
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.true",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *cmp_false
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.false",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *cmp_end
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.end",
                                   llvm_visitor->get_current_function());

    // Normalize operands types
    vlhs = llvm_visitor->ir_builder->CreatePointerCast(
        vlhs, llvm_visitor->llvm_types.ptr_i8);
    vrhs = llvm_visitor->ir_builder->CreatePointerCast(
        vrhs, llvm_visitor->llvm_types.ptr_i8);

    // A first loop does the comparison between the prefix
    llvm::Value *idx_var
        = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
    llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0),
                                          idx_var);
    llvm_visitor->ir_builder->CreateBr(loop_check);

    llvm_visitor->set_current_block(loop_check);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), len_min),
        loop_body,
        loop_end);

    llvm_visitor->set_current_block(loop_body);

    llvm::Value *lhs_offset = llvm_visitor->ir_builder->CreateGEP(
        vlhs, { llvm_visitor->ir_builder->CreateLoad(idx_var) });
    llvm::Value *lhs_char = llvm_visitor->ir_builder->CreateLoad(lhs_offset);

    llvm::Value *rhs_offset = llvm_visitor->ir_builder->CreateGEP(
        vrhs, { llvm_visitor->ir_builder->CreateLoad(idx_var) });
    llvm::Value *rhs_char = llvm_visitor->ir_builder->CreateLoad(rhs_offset);

    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpEQ(lhs_char, rhs_char),
        loop_body_next,
        loop_body_check);

    llvm_visitor->set_current_block(loop_body_check);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLE(lhs_char, rhs_char),
        cmp_true,
        cmp_false);

    llvm_visitor->set_current_block(loop_body_next);
    llvm_visitor->ir_builder->CreateStore(
        llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateLoad(idx_var),
            llvm_visitor->get_integer_value_64(1)),
        idx_var);
    llvm_visitor->ir_builder->CreateBr(loop_check);

    llvm_visitor->set_current_block(loop_end);

    // A second loop makes sure that the remainder of the longest
    // string is all blanks
    llvm::Value *checked_character = llvm_visitor->ir_builder->CreateSelect(
        llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), len_lhs),
        vlhs,
        vrhs);
    llvm::Value *len_max = llvm_visitor->ir_builder->CreateSelect(
        llvm_visitor->ir_builder->CreateICmpSGT(len_lhs, len_rhs),
        len_lhs,
        len_rhs);
    llvm_visitor->ir_builder->CreateBr(tail_check);

    llvm_visitor->set_current_block(tail_check);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), len_max),
        tail_body,
        cmp_true);

    llvm_visitor->set_current_block(tail_body);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpEQ(
            llvm_visitor->ir_builder->CreateLoad(
                llvm_visitor->ir_builder->CreateGEP(
                    llvm_visitor->ir_builder->CreatePointerCast(
                        checked_character, llvm_visitor->llvm_types.ptr_i8),
                    { llvm_visitor->ir_builder->CreateLoad(idx_var) })),
            llvm_visitor->get_integer_value_N(
                ' ', llvm_visitor->llvm_types.i8, 8)),
        tail_body_next,
        tail_nonempty);

    llvm_visitor->set_current_block(tail_body_next);
    llvm_visitor->ir_builder->CreateStore(
        llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateLoad(idx_var),
            llvm_visitor->get_integer_value_64(1)),
        idx_var);
    llvm_visitor->ir_builder->CreateBr(tail_check);

    llvm_visitor->set_current_block(tail_nonempty);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLT(len_lhs, len_rhs),
        cmp_true,
        cmp_false);

    llvm_visitor->set_current_block(cmp_true);
    llvm::Value *result_true
        = llvm_visitor->get_integer_value_N(1, llvm_visitor->llvm_types.i1, 1);
    llvm_visitor->ir_builder->CreateBr(cmp_end);

    llvm_visitor->set_current_block(cmp_false);
    llvm::Value *result_false
        = llvm_visitor->get_integer_value_N(0, llvm_visitor->llvm_types.i1, 1);
    llvm_visitor->ir_builder->CreateBr(cmp_end);

    llvm_visitor->set_current_block(cmp_end);
    llvm::Value *value
        = llvm_visitor->ir_builder->CreatePHI(llvm_visitor->llvm_types.i1, 2);
    llvm::cast<llvm::PHINode>(value)->addIncoming(result_true, cmp_true);
    llvm::cast<llvm::PHINode>(value)->addIncoming(result_false, cmp_false);

    value = llvm_visitor->ir_builder->CreateZExtOrTrunc(
        value, llvm_visitor->get_llvm_type(node.get_type()));

    return value;
}

void FortranVisitorLLVMExpression::visit(const Nodecl::LowerOrEqualThan &node)
{
    CharacterCompareLE character_compare_le(llvm_visitor, node);
    binary_comparison(node,
                      CREATOR_CMP(CreateICmpSLE, node),
                      CREATOR_CMP(CreateFCmpOLE, node),
                      INVALID_OP,
                      character_compare_le);
}

void FortranVisitorLLVMExpression::visit(const Nodecl::GreaterThan &node)
{
    auto character_compare_gt = [&, this](Nodecl::NodeclBase lhs,
                                          Nodecl::NodeclBase rhs,
                                          llvm::Value *vlhs,
                                          llvm::Value *vrhs) {
        CharacterCompareLT character_compare_lt(llvm_visitor, node);
        // a > b computed as b < a
        return character_compare_lt(rhs, lhs, vrhs, vlhs);
    };
    binary_comparison(node,
                      CREATOR_CMP(CreateICmpSGT, node),
                      CREATOR_CMP(CreateFCmpOGT, node),
                      INVALID_OP,
                      character_compare_gt);
}

void FortranVisitorLLVMExpression::visit(const Nodecl::GreaterOrEqualThan &node)
{
    auto character_compare_ge = [&, this](Nodecl::NodeclBase lhs,
                                          Nodecl::NodeclBase rhs,
                                          llvm::Value *vlhs,
                                          llvm::Value *vrhs) -> llvm::Value * {
        CharacterCompareLE character_compare_le(llvm_visitor, node);
        // a >= b computed as b <= a
        return character_compare_le(rhs, lhs, vrhs, vlhs);
    };
    binary_comparison(node,
                      CREATOR_CMP(CreateICmpSGE, node),
                      CREATOR_CMP(CreateFCmpOGE, node),
                      INVALID_OP,
                      character_compare_ge);
}

FortranVisitorLLVMExpression::CharacterCompareEQ::CharacterCompareEQ(
    FortranLLVM *llvm_visitor, Nodecl::NodeclBase node)
    : llvm_visitor(llvm_visitor), node(node)
{
}

llvm::Value *FortranVisitorLLVMExpression::CharacterCompareEQ::operator()(
    Nodecl::NodeclBase lhs,
    Nodecl::NodeclBase rhs,
    llvm::Value *vlhs,
    llvm::Value *vrhs)
{
    llvm::Value *len_lhs
        = llvm_visitor->eval_length_of_character(lhs.get_type());
    llvm::Value *len_rhs
        = llvm_visitor->eval_length_of_character(rhs.get_type());

    llvm::Value *len_min = llvm_visitor->ir_builder->CreateSelect(
        llvm_visitor->ir_builder->CreateICmpSLT(len_lhs, len_rhs),
        len_lhs,
        len_rhs);

    llvm::BasicBlock *loop_check
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.check",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_body
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.body",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_body_next
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.body.next",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *loop_end
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.loop.end",
                                   llvm_visitor->get_current_function());

    llvm::BasicBlock *tail_check
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.loop.check",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *tail_body
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.loop.body",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *tail_body_next
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.tail.loop.body.next",
                                   llvm_visitor->get_current_function());

    llvm::BasicBlock *cmp_true
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.true",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *cmp_false
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.false",
                                   llvm_visitor->get_current_function());
    llvm::BasicBlock *cmp_end
        = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                   "character.cmp.end",
                                   llvm_visitor->get_current_function());

    // Normalize operands types
    vlhs = llvm_visitor->ir_builder->CreatePointerCast(
        vlhs, llvm_visitor->llvm_types.ptr_i8);
    vrhs = llvm_visitor->ir_builder->CreatePointerCast(
        vrhs, llvm_visitor->llvm_types.ptr_i8);

    // A first loop does the comparison between the prefix
    llvm::Value *idx_var
        = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
    llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0),
                                          idx_var);
    llvm_visitor->ir_builder->CreateBr(loop_check);

    llvm_visitor->set_current_block(loop_check);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), len_min),
        loop_body,
        loop_end);

    llvm_visitor->set_current_block(loop_body);

    llvm::Value *lhs_offset = llvm_visitor->ir_builder->CreateGEP(
        vlhs, { llvm_visitor->ir_builder->CreateLoad(idx_var) });
    llvm::Value *lhs_char = llvm_visitor->ir_builder->CreateLoad(lhs_offset);

    llvm::Value *rhs_offset = llvm_visitor->ir_builder->CreateGEP(
        vrhs, { llvm_visitor->ir_builder->CreateLoad(idx_var) });
    llvm::Value *rhs_char = llvm_visitor->ir_builder->CreateLoad(rhs_offset);

    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpEQ(lhs_char, rhs_char),
        loop_body_next,
        cmp_false);

    llvm_visitor->set_current_block(loop_body_next);
    llvm_visitor->ir_builder->CreateStore(
        llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateLoad(idx_var),
            llvm_visitor->get_integer_value_64(1)),
        idx_var);
    llvm_visitor->ir_builder->CreateBr(loop_check);

    llvm_visitor->set_current_block(loop_end);

    // A second loop makes sure that the remainder of the longest
    // string is all blanks
    llvm::Value *checked_character = llvm_visitor->ir_builder->CreateSelect(
        llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), len_lhs),
        vlhs,
        vrhs);
    llvm::Value *len_max = llvm_visitor->ir_builder->CreateSelect(
        llvm_visitor->ir_builder->CreateICmpSGT(len_lhs, len_rhs),
        len_lhs,
        len_rhs);
    llvm_visitor->ir_builder->CreateBr(tail_check);

    llvm_visitor->set_current_block(tail_check);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var), len_max),
        tail_body,
        cmp_true);

    llvm_visitor->set_current_block(tail_body);
    llvm_visitor->ir_builder->CreateCondBr(
        llvm_visitor->ir_builder->CreateICmpEQ(
            llvm_visitor->ir_builder->CreateLoad(
                llvm_visitor->ir_builder->CreateGEP(
                    llvm_visitor->ir_builder->CreatePointerCast(
                        checked_character, llvm_visitor->llvm_types.ptr_i8),
                    { llvm_visitor->ir_builder->CreateLoad(idx_var) })),
            llvm_visitor->get_integer_value_N(
                ' ', llvm_visitor->llvm_types.i8, 8)),
        tail_body_next,
        cmp_false);

    llvm_visitor->set_current_block(tail_body_next);
    llvm_visitor->ir_builder->CreateStore(
        llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateLoad(idx_var),
            llvm_visitor->get_integer_value_64(1)),
        idx_var);
    llvm_visitor->ir_builder->CreateBr(tail_check);

    llvm_visitor->set_current_block(cmp_true);
    llvm::Value *result_true
        = llvm_visitor->get_integer_value_N(1, llvm_visitor->llvm_types.i1, 1);
    llvm_visitor->ir_builder->CreateBr(cmp_end);

    llvm_visitor->set_current_block(cmp_false);
    llvm::Value *result_false
        = llvm_visitor->get_integer_value_N(0, llvm_visitor->llvm_types.i1, 1);
    llvm_visitor->ir_builder->CreateBr(cmp_end);

    llvm_visitor->set_current_block(cmp_end);
    llvm::Value *value
        = llvm_visitor->ir_builder->CreatePHI(llvm_visitor->llvm_types.i1, 2);
    llvm::cast<llvm::PHINode>(value)->addIncoming(result_true, cmp_true);
    llvm::cast<llvm::PHINode>(value)->addIncoming(result_false, cmp_false);

    value = llvm_visitor->ir_builder->CreateZExtOrTrunc(
        value, llvm_visitor->get_llvm_type(node.get_type()));

    return value;
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Equal &node)
{
    auto creator_complex = [&, this](Nodecl::NodeclBase,
                                     Nodecl::NodeclBase,
                                     llvm::Value *lhs,
                                     llvm::Value *rhs) -> llvm::Value * {
        llvm::Value *lhs_real
            = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(0));
        llvm::Value *lhs_imag
            = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(1));

        llvm::Value *rhs_real
            = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(0));
        llvm::Value *rhs_imag
            = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(1));

        return llvm_visitor->ir_builder->CreateZExtOrTrunc(
            llvm_visitor->ir_builder->CreateSelect(
                llvm_visitor->ir_builder->CreateFCmpOEQ(lhs_real, rhs_real),
                llvm_visitor->ir_builder->CreateFCmpOEQ(lhs_imag, rhs_imag),
                llvm_visitor->get_integer_value_N(
                    0, llvm_visitor->llvm_types.i1, 1)),
            llvm_visitor->get_llvm_type(node.get_type()));
    };

    CharacterCompareEQ character_compare_eq(llvm_visitor, node);

    binary_comparison(node,
                      CREATOR_CMP(CreateICmpEQ, node),
                      CREATOR_CMP(CreateFCmpOEQ, node),
                      creator_complex,
                      character_compare_eq);
}

void FortranVisitorLLVMExpression::visit(const Nodecl::Different &node)
{
    auto creator_complex = [&, this](Nodecl::NodeclBase,
                                     Nodecl::NodeclBase,
                                     llvm::Value *lhs,
                                     llvm::Value *rhs) -> llvm::Value * {
        llvm::Value *lhs_real
            = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(0));
        llvm::Value *lhs_imag
            = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(1));

        llvm::Value *rhs_real
            = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(0));
        llvm::Value *rhs_imag
            = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(1));

        return llvm_visitor->ir_builder->CreateZExtOrTrunc(
            llvm_visitor->ir_builder->CreateSelect(
                llvm_visitor->ir_builder->CreateFCmpONE(lhs_real, rhs_real),
                llvm_visitor->ir_builder->CreateFCmpONE(lhs_imag, rhs_imag),
                llvm_visitor->get_integer_value_N(
                    0, llvm_visitor->llvm_types.i1, 1)),
            llvm_visitor->get_llvm_type(node.get_type()));
    };

    auto character_compare_ne = [&, this](Nodecl::NodeclBase lhs,
                                          Nodecl::NodeclBase rhs,
                                          llvm::Value *vlhs,
                                          llvm::Value *vrhs) -> llvm::Value * {
        CharacterCompareEQ compare_eq(llvm_visitor, node);

        llvm::Value *v = llvm_visitor->ir_builder->CreateZExtOrTrunc(
            llvm_visitor->ir_builder->CreateSelect(
                llvm_visitor->ir_builder->CreateZExtOrTrunc(
                    compare_eq(lhs, rhs, vlhs, vrhs),
                    llvm_visitor->llvm_types.i1),
                llvm_visitor->get_integer_value_N(
                    0, llvm_visitor->llvm_types.i1, 1),
                llvm_visitor->get_integer_value_N(
                    1, llvm_visitor->llvm_types.i1, 1)),
            llvm_visitor->get_llvm_type(node.get_type()));
        return v;
    };

    binary_comparison(node,
                      CREATOR_CMP(CreateICmpNE, node),
                      CREATOR_CMP(CreateFCmpONE, node),
                      creator_complex,
                      character_compare_ne);
}
#undef CREATOR
#undef UNIMPLEMENTED_OP
#undef INVALID_OP

void FortranVisitorLLVMExpression::visit(const Nodecl::Concat &node)
{
    auto create_concat = [&, this](Nodecl::NodeclBase lhs,
                                   Nodecl::NodeclBase rhs,
                                   llvm::Value *vlhs,
                                   llvm::Value *vrhs) -> llvm::Value * {
        TL::Type lhs_type = lhs.get_type();
        while (lhs_type.is_fortran_array())
            lhs_type = lhs_type.array_element();

        // TODO - CHARACTER(LEN=*) is special as it uses a hidden parameter
        llvm::Value *lhs_elements
            = llvm_visitor->eval_length_of_character(lhs_type);

        TL::Type rhs_type = rhs.get_type();
        while (rhs_type.is_fortran_array())
            rhs_type = rhs_type.array_element();

        llvm::Value *rhs_elements
            = llvm_visitor->eval_length_of_character(rhs_type);

        llvm::Value *total_elements
            = llvm_visitor->ir_builder->CreateAdd(lhs_elements, rhs_elements);

        // We only support CHARACTER(KIND=1) so i8 is fine here
        llvm::Value *result_addr = llvm_visitor->ir_builder->CreateAlloca(
            llvm_visitor->llvm_types.i8, total_elements);

        llvm_visitor->ir_builder->CreateMemCpy(
            result_addr, vlhs, lhs_elements, 1);

        llvm::Value *rhs_offset = llvm_visitor->ir_builder->CreateGEP(
            result_addr, { lhs_elements });

        llvm_visitor->ir_builder->CreateMemCpy(
            rhs_offset, vrhs, rhs_elements, 1);

        return result_addr;
    };
    binary_operator(node, create_concat);
}
// void visit(const Nodecl::ClassMemberAccess& node);

void FortranVisitorLLVMExpression::visit(const Nodecl::Range &node)
{
    // Do nothing
}

void FortranVisitorLLVMExpression::visit(const Nodecl::StringLiteral &node)
{
    std::string str = node.get_text();
    str = str.substr(1, str.size() - 2);
    value = llvm_visitor->ir_builder->CreateGlobalStringPtr(str);
}

void FortranVisitorLLVMExpression::visit(const Nodecl::IntegerLiteral &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    value = llvm_visitor->get_integer_value(
        // FIXME: INTEGER(16)
        const_value_cast_to_8(node.get_constant()),
        node.get_type());
}

void FortranVisitorLLVMExpression::visit(const Nodecl::FloatingLiteral &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    TL::Type t = node.get_type();
    if (t.is_float())
    {
        value = llvm::ConstantFP::get(
            llvm_visitor->llvm_context,
            llvm::APFloat(const_value_cast_to_float(node.get_constant())));
    }
    else if (t.is_double())
    {
        value = llvm::ConstantFP::get(
            llvm_visitor->llvm_context,
            llvm::APFloat(const_value_cast_to_double(node.get_constant())));
    }
    else
    {
        internal_error("Unexpected type '%s'",
                       print_declarator(t.get_internal_type()));
    }
}
// void visit(const Nodecl::Dereference& node);
// void visit(const Nodecl::Reference& node);
void FortranVisitorLLVMExpression::visit(
    const Nodecl::ParenthesizedExpression &node)
{
    FortranLLVM::TrackLocation loc(llvm_visitor, node);

    // FIXME: Make sure the code preserves the integrity of the parentheses
    // (i.e. (A + B) + C does not become A + (B + C)
    walk(node.get_nest());
}

// FIXME - Use GEP when possible
llvm::Value *FortranVisitorLLVMExpression::
    address_of_subscripted_array_no_descriptor(
        const Nodecl::ArraySubscript &node)
{
    Nodecl::NodeclBase subscripted = node.get_subscripted();
    TL::Type subscripted_type = subscripted.get_type();
    TL::Type subscripted_type_noref = subscripted_type.no_ref();

    Nodecl::List subscripts = node.get_subscripts().as<Nodecl::List>();
    std::vector<llvm::Value *> offset_list, size_list;
    offset_list.reserve(subscripts.size());
    size_list.reserve(subscripts.size());

    TL::Type current_array_type = subscripted_type_noref;
    for (Nodecl::NodeclBase index : subscripts)
    {
        Nodecl::NodeclBase lower, upper;
        current_array_type.array_get_bounds(lower, upper);
        llvm::Value *val_lower = llvm_visitor->ir_builder->CreateSExtOrTrunc(
            llvm_visitor->eval_expression(lower), llvm_visitor->llvm_types.i64);
        llvm::Value *val_upper = llvm_visitor->ir_builder->CreateSExtOrTrunc(
            llvm_visitor->eval_expression(upper), llvm_visitor->llvm_types.i64);

        ERROR_CONDITION(index.is<Nodecl::Range>(), "Invalid subscript here", 0);
        llvm::Value *val_idx = llvm_visitor->ir_builder->CreateSExtOrTrunc(
            llvm_visitor->eval_expression(index), llvm_visitor->llvm_types.i64);

        llvm::Value *val_offset
            = llvm_visitor->ir_builder->CreateSub(val_idx, val_lower);
        offset_list.push_back(val_offset);

        llvm::Value *val_size = llvm_visitor->ir_builder->CreateAdd(
            llvm_visitor->ir_builder->CreateSub(val_upper, val_lower),
            llvm_visitor->get_integer_value_64(1));
        size_list.push_back(val_size);

        current_array_type = current_array_type.array_element();
    }


    // Compute an offset in elements using Horner's rule.
    std::vector<llvm::Value *>::iterator it_offsets = offset_list.begin();
    std::vector<llvm::Value *>::iterator it_sizes = size_list.begin();

    llvm::Value *val_addr = *it_offsets;
    it_offsets++;
    it_sizes++;

    while (it_offsets != offset_list.end() && it_sizes != size_list.end())
    {
        val_addr = llvm_visitor->ir_builder->CreateAdd(
            *it_offsets,
            llvm_visitor->ir_builder->CreateMul(*it_sizes, val_addr));

        it_offsets++;
        it_sizes++;
    }
    ERROR_CONDITION(it_offsets != offset_list.end()
                        || it_sizes != size_list.end(),
                    "Lists do not match",
                    0);

    // Now multiply by the size of the type to get an offset in bytes
    ERROR_CONDITION(current_array_type.is_fortran_array(),
                    "Should not be an array here",
                    0);

    llvm::Value *base_addr = llvm_visitor->eval_expression(subscripted);

    return llvm_visitor->ir_builder->CreateGEP(base_addr, { val_addr });
}

llvm::Value *FortranVisitorLLVMExpression::
    address_of_subscripted_array_descriptor(const Nodecl::ArraySubscript &node)
{
    Nodecl::NodeclBase subscripted = node.get_subscripted();
    TL::Type subscripted_type = subscripted.get_type();
    TL::Type subscripted_type_noref = subscripted_type.no_ref();

    llvm::Value *descriptor_addr = llvm_visitor->eval_expression(subscripted);

    Nodecl::List subscripts_tree = node.get_subscripts().as<Nodecl::List>();
    // Reverse subscripts as in the tree are represented in the C order
    TL::ObjectList<Nodecl::NodeclBase> subscripts(subscripts_tree.rbegin(),
                                                  subscripts_tree.rend());

    llvm::Value *linear_index = nullptr;
    int rank = 0;
    for (TL::ObjectList<Nodecl::NodeclBase>::iterator it = subscripts.begin();
         it != subscripts.end();
         it++, rank++)
    {
        llvm::Value *current = llvm_visitor->ir_builder->CreateMul(
            llvm_visitor->ir_builder->CreateZExtOrTrunc(
                llvm_visitor->eval_expression(*it),
                llvm_visitor->llvm_types.i64),
            llvm_visitor->ir_builder->CreateLoad(
                llvm_visitor->array_descriptor_addr_dim_stride(descriptor_addr,
                                                               rank)));

        if (linear_index == nullptr)
            linear_index = current;
        else
            linear_index
                = llvm_visitor->ir_builder->CreateAdd(linear_index, current);
    }

    llvm::Value *offset_value = llvm_visitor->ir_builder->CreateLoad(
        llvm_visitor->array_descriptor_addr_offset(descriptor_addr));

    linear_index
        = llvm_visitor->ir_builder->CreateAdd(linear_index, offset_value);

    TL::Type element_type = subscripted_type_noref.fortran_array_base_element();

    llvm::Value *base_address = llvm_visitor->ir_builder->CreateLoad(
        llvm_visitor->array_descriptor_addr_base_addr(descriptor_addr));
    base_address = llvm_visitor->ir_builder->CreatePointerCast(
        base_address,
        llvm::PointerType::get(llvm_visitor->get_llvm_type(element_type),
                               /* AddressSpace */ 0));

    return llvm_visitor->ir_builder->CreateGEP(base_address, { linear_index });
}

void FortranVisitorLLVMExpression::visit(const Nodecl::ArraySubscript &node)
{
    Nodecl::NodeclBase subscripted = node.get_subscripted();
    TL::Type subscripted_type = subscripted.get_type();
    TL::Type subscripted_type_noref = subscripted_type.no_ref();

    // FIXME: Substrings use this node too
    ERROR_CONDITION(!subscripted_type_noref.is_fortran_array(),
                    "Expecting an array here",
                    0);

    if (subscripted_type_noref.array_requires_descriptor())
    {
        if (node.get_type().no_ref().is_fortran_array()
            && node.get_type().no_ref().array_is_region())
            internal_error("Not yet implemented", 0);

        value = address_of_subscripted_array_descriptor(node);
    }
    else if (node.get_type().no_ref().is_fortran_array()
             && node.get_type().no_ref().array_is_region())
    {
        llvm::Value *subscripted_val
            = llvm_visitor->eval_expression(subscripted);
        // We will compute everything based on the region described in the
        // type
        value = subscripted_val;
    }
    else
    {
        // We compute the address of this element
        value = address_of_subscripted_array_no_descriptor(node);
    }
}

void FortranVisitorLLVMExpression::visit(const Nodecl::FunctionCall &node)
{
    Nodecl::NodeclBase called = node.get_called();
    Nodecl::List arguments = node.get_arguments().as<Nodecl::List>();

    ERROR_CONDITION(
        !called.is<Nodecl::Symbol>(), "We can only call functions", 0);
    TL::Symbol called_sym = called.get_symbol();

    if (called_sym.is_builtin())
        return implement_builtin_call(node);

    TL::Type called_type = called_sym.get_type();
    ERROR_CONDITION(
        !called_type.is_function(), "Expecting a function type here", 0);

    bool call_without_interface
        = called_type.lacks_prototype()
          // This check is required because the FE updates the symbol of the
          // call but does not update the argument list, so the type may not
          // be prototyped.  Fortunately, the FE has kept the original
          // unprototyped in the alternate name so we can query that one
          // instead and tell that, after all, this call is unprototyped.
          // Ideally the FE should update the arguments of the call but it is
          // not doing that yet.
          || (!node.get_alternate_name().is_null()
              && node.get_alternate_name().get_symbol().is_valid()
              && node.get_alternate_name().get_symbol().get_type().is_valid()
              && node.get_alternate_name()
                     .get_symbol()
                     .get_type()
                     .lacks_prototype());

    if (called_sym.is_from_module())
    {
        TL::Symbol from_module = called_sym.from_module();
        llvm_visitor->dbg_builder->createImportedModule(
            llvm_visitor->dbg_info.function,
            llvm_visitor->get_module(from_module.get_name()),
            called_sym.get_line());
    }

    std::string mangled_name
        = fortran_mangle_symbol(called_sym.get_internal_symbol());
    llvm::FunctionType *function_type = llvm::cast<llvm::FunctionType>(
        llvm_visitor->get_llvm_type(called_sym.get_type()));

    llvm::Constant *c = llvm_visitor->current_module->getOrInsertFunction(
        mangled_name,
        function_type,
        /* no attributes so far */ llvm::AttributeSet());
    llvm::Function *fun = llvm::cast<llvm::Function>(c);

    // Make sure we use the right unprototyped type here. The FE does not
    // set the number of arguments so we need to compute a correctly
    // numbered unprototyped type here.
    if (call_without_interface)
    {
        TL::Type ret_type;
        if (called_type.lacks_prototype())
        {
            ret_type = called_type.returns();
        }
        else
        {
            ret_type
                = node.get_alternate_name().get_symbol().get_type().returns();
        }
        // Note, this will create a function with int parameters. Make sure they
        // are not used.
        called_type = ::get_nonproto_function_type(ret_type.get_internal_type(),
                                                   arguments.size());
        // This will create a function with i8* parameters.
        function_type = llvm::cast<llvm::FunctionType>(
            llvm_visitor->get_llvm_type(called_type));
    }

    TL::ObjectList<TL::Type> parameters = called_type.parameters();

    std::vector<llvm::Value *> val_arguments;
    val_arguments.reserve(parameters.size());

    Nodecl::List::iterator it_arg = arguments.begin();
    TL::ObjectList<TL::Type>::iterator it_param = parameters.begin();

    while (it_arg != arguments.end() && it_param != parameters.end())
    {
        Nodecl::NodeclBase arg = *it_arg;
        ERROR_CONDITION(!arg.is<Nodecl::FortranActualArgument>(),
                        "Unexpected node '%s'",
                        ast_print_node_type(arg.get_kind()));
        arg = arg.as<Nodecl::FortranActualArgument>().get_argument();

        llvm::Value *varg = nullptr;
        if (call_without_interface)
        {
            varg = llvm_visitor->eval_expression_to_memory(arg);
            varg = llvm_visitor->ir_builder->CreateBitCast(
                varg, llvm_visitor->llvm_types.ptr_i8);
        }
        else
        {
            varg = llvm_visitor->eval_expression(arg);
            if (it_param->no_ref().is_fortran_array())
            {
                if (!arg.get_type().no_ref().is_fortran_array()
                    || (!it_param->no_ref().array_requires_descriptor()
                        && !arg.get_type()
                                .no_ref()
                                .array_requires_descriptor()))
                {
                    TL::Type array_type = it_param->no_ref();
                    TL::Type element_type = array_type.array_base_element();

                    // Cast to a pointer of the element
                    varg = llvm_visitor->ir_builder->CreateBitCast(
                        varg,
                        llvm::PointerType::get(
                            llvm_visitor->get_llvm_type(element_type),
                            /* AddressSpace */ 0));
                }
                else if (!arg.get_type().no_ref().array_requires_descriptor()
                         && it_param->no_ref().array_requires_descriptor())
                {
                    // We need to create a descriptor here
                    llvm::Type *descriptor_type
                        = llvm_visitor->get_gfortran_array_descriptor_type(
                            arg.get_type().no_ref());
                    llvm::Value *descriptor_addr
                        = llvm_visitor->ir_builder->CreateAlloca(
                            descriptor_type, nullptr);

                    llvm::Value *base_address
                        = llvm_visitor->ir_builder->CreatePointerCast(
                            varg, llvm_visitor->llvm_types.ptr_i8);

                    llvm_visitor->fill_descriptor_info(
                        arg.get_type().no_ref(), descriptor_addr, base_address);

                    varg = descriptor_addr;
                }
                else if (arg.get_type().no_ref().array_requires_descriptor()
                         && !it_param->no_ref().array_requires_descriptor())
                {
                    // If the array is CONTIGUOUS we can use the buffer
                    // directly, otherwise we will need to create a
                    // temporary
                    internal_error("Not yet implemented", 0);
                }
                else if (arg.get_type().no_ref().array_requires_descriptor()
                         && it_param->no_ref().array_requires_descriptor())
                {
                    // Nothing special is required, we already have a proper
                    // descriptor
                }
                else
                {
                    internal_error("Code unreachable", 0);
                }
            }
        }

        val_arguments.push_back(varg);

        it_arg++;
        it_param++;
    }
    ERROR_CONDITION(it_arg != arguments.end() || it_param != parameters.end(),
                    "Mismatch between arguments and parameters",
                    0);

    if (call_without_interface)
    {
        llvm::Value *bitcast = llvm_visitor->ir_builder->CreateBitCast(
            fun, llvm::PointerType::get(function_type, /* AddressSpace */ 0));
        value = llvm_visitor->ir_builder->CreateCall(bitcast, val_arguments);
    }
    else
    {
        value = llvm_visitor->ir_builder->CreateCall(fun, val_arguments);
    }
}


llvm::Value *FortranLLVM::eval_expression(Nodecl::NodeclBase n)
{
    FortranVisitorLLVMExpression v(this);
    v.walk(n);
    return v.get_value();
}

llvm::Value *FortranLLVM::make_temporary(llvm::Value *v)
{
    llvm::Value *tmp = ir_builder->CreateAlloca(v->getType());
    ir_builder->CreateStore(v, tmp);
    return tmp;
}

llvm::Value *FortranLLVM::eval_expression_to_memory(Nodecl::NodeclBase n)
{
    FortranVisitorLLVMExpression v(this);
    v.walk(n);
    llvm::Value *result = v.get_value();
    if (!n.get_type().is_any_reference())
        result = make_temporary(result);

    return result;
}

llvm::Value *FortranLLVM::eval_sizeof(TL::Type t)
{
    return ir_builder->CreateZExtOrTrunc(eval_sizeof_64(t), llvm_types.i32);
}
llvm::Value *FortranLLVM::eval_sizeof(Nodecl::NodeclBase n)
{
    return eval_sizeof(n.get_type());
}

llvm::Value *FortranLLVM::eval_sizeof_64(Nodecl::NodeclBase n)
{
    return eval_sizeof(n.get_type());
}

llvm::Value *FortranLLVM::eval_sizeof_64(TL::Type t)
{
    if (t.is_fortran_array())
    {
        return ir_builder->CreateMul(eval_size_of_array(t),
                                     eval_sizeof_64(t.array_base_element()));
    }
    else
    {
        return get_integer_value_64(t.no_ref().get_size());
    }
}

llvm::Value *FortranLLVM::eval_size_of_dimension(TL::Type t)
{
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type", 0);

    llvm::Value *current_size = nullptr;
    if (t.array_requires_descriptor())
    {
        internal_error("Not yet implemented", 0);
    }
    else
    {
        current_size = eval_expression(t.array_get_size());
    }

    return current_size;
}

llvm::Value *FortranLLVM::eval_size_of_array(TL::Type t)
{
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type", 0);
    if (t.array_requires_descriptor())
    {
        internal_error("Not yet implemented", 0);
    }
    else
    {
        llvm::Value *val_size = nullptr;
        while (t.is_fortran_array())
        {
            llvm::Value *current_size = eval_size_of_dimension(t);

            if (val_size == nullptr)
                val_size = current_size;
            else
                val_size = ir_builder->CreateMul(val_size, current_size);

            t = t.array_element();
        }

        return val_size;
    }
}

llvm::Value *FortranLLVM::eval_length_of_character(TL::Type t)
{
    // FIXME - CHARACTER(LEN=*)
    ERROR_CONDITION(!t.is_fortran_character(), "Invalid type", 0);
    llvm::Value *val_size = eval_expression(t.array_get_size());

    return val_size;
}

llvm::Value *FortranLLVM::eval_elements_of_dimension(Nodecl::NodeclBase expr,
                                                     TL::Type t,
                                                     llvm::Value *addr,
                                                     int dimension)
{
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type", 0);

    llvm::Value *vlower = nullptr;
    llvm::Value *vupper = nullptr;
    llvm::Value *vstride = nullptr;

    if (t.array_requires_descriptor())
    {
        ERROR_CONDITION(addr == nullptr,
                        "Need an address when the array has a descriptor",
                        0);
        if (!expr.is_null() && array_expression_will_use_unit_stride(expr))
            vstride = get_integer_value_64(1);
        else
            vstride = ir_builder->CreateLoad(
                array_descriptor_addr_dim_stride(addr, dimension));
        vlower = ir_builder->CreateLoad(
            array_descriptor_addr_dim_lower_bound(addr, dimension));
        vupper = ir_builder->CreateLoad(
            array_descriptor_addr_dim_upper_bound(addr, dimension));
    }
    else if (t.array_is_region())
    {
        // We cannot use TL::Type functions here because they are
        // oblivious of the step (they assume step=1)
        Nodecl::NodeclBase lower
            = array_type_get_region_lower_bound(t.get_internal_type());
        Nodecl::NodeclBase upper
            = array_type_get_region_upper_bound(t.get_internal_type());
        Nodecl::NodeclBase stride
            = array_type_get_region_stride(t.get_internal_type());

        vlower = eval_expression(lower);
        vupper = eval_expression(upper);
        vstride = eval_expression(stride);
    }
    else // Plain array
    {
        Nodecl::NodeclBase lower
            = array_type_get_array_lower_bound(t.get_internal_type());
        Nodecl::NodeclBase upper
            = array_type_get_array_upper_bound(t.get_internal_type());

        vlower = eval_expression(lower);
        vupper = eval_expression(upper);
        vstride = get_integer_value_64(1);
    }

    llvm::Value *current_size = ir_builder->CreateSDiv(
        ir_builder->CreateAdd(ir_builder->CreateSub(vupper, vlower), vstride),
        vstride);

    return current_size;
}

llvm::Value *FortranLLVM::eval_elements_of_array(Nodecl::NodeclBase expr,
                                                 TL::Type t,
                                                 llvm::Value *addr)
{
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type", 0);
    llvm::Value *val_size = nullptr;
    int dimension = 0;
    while (t.is_fortran_array())
    {
        llvm::Value *current_size
            = eval_elements_of_dimension(expr, t, addr, dimension);

        if (val_size == nullptr)
            val_size = current_size;
        else
            val_size = ir_builder->CreateMul(val_size, current_size);

        t = t.array_element();
        dimension++;
    }

    return val_size;
}
}
