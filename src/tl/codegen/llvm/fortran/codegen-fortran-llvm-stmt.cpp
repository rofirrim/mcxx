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
#include "codegen-fortran-llvm-expr.hpp"
#include "cxx-cexpr.h"
#include "fortran03-typeutils.h"

namespace Codegen {

void FortranLLVM::visit(const Nodecl::CompoundStatement &node)
{
    FortranLLVM::TrackLocation loc(this, node);
    walk(node.get_statements());
}

void FortranLLVM::visit(const Nodecl::ExpressionStatement &node)
{
    FortranLLVM::TrackLocation loc(this, node);
    eval_expression(node.get_nest());
}

void FortranLLVM::visit(const Nodecl::EmptyStatement &)
{
    // Do nothing
}

void FortranLLVM::visit(const Nodecl::FortranStopStatement &node)
{
    Nodecl::NodeclBase expr = node.get_stop_code(); 
    ERROR_CONDITION(!expr.get_type().is_signed_integral(),
        "Unsupported type '%s' in STOP expression", print_declarator(expr.get_type().get_internal_type()));
    llvm::Value *v = eval_expression(expr);
    v = ir_builder->CreateSExtOrTrunc(v, llvm_types.i32);

    ir_builder->CreateCall(gfortran_rt.stop_int.get(), std::vector<llvm::Value*>(1, v));
}

void FortranLLVM::visit(const Nodecl::ObjectInit& node)
{
    TL::Symbol sym = node.get_symbol();

    if (sym.is_variable()
            && sym.is_saved_expression())
    {
        llvm::Value *vlhs = get_value(sym);
        llvm::Value *vrhs = eval_expression(sym.get_value());

        ir_builder->CreateStore(vrhs, vlhs);
    }
    else
    {
        std::cerr << "Unhandled object init. Symbol = " << node.get_symbol().get_name() << " " << symbol_kind_name(node.get_symbol().get_internal_symbol()) << std::endl;
    }
}

void FortranLLVM::visit(const Nodecl::IfElseStatement& node)
{
    Nodecl::NodeclBase condition = node.get_condition();
    Nodecl::NodeclBase then_node = node.get_then();
    Nodecl::NodeclBase else_node = node.get_else();

    llvm::Value *cond_val = ir_builder->CreateZExtOrTrunc(
            eval_expression(condition),
            llvm_types.i1);

    llvm::BasicBlock * block_true = llvm::BasicBlock::Create(llvm_context, "if.true", get_current_function());
    llvm::BasicBlock * block_end = llvm::BasicBlock::Create(llvm_context, "if.end", get_current_function());

    llvm::BasicBlock * block_false = block_end;
    if (!else_node.is_null())
    {
        block_false = llvm::BasicBlock::Create(llvm_context, "if.else", get_current_function());
    }

    ir_builder->CreateCondBr(cond_val, block_true, block_false);

    set_current_block(block_true);
    walk(then_node);
    ir_builder->CreateBr(block_end);

    if (!else_node.is_null())
    {
        set_current_block(block_false);
        walk(else_node);
        ir_builder->CreateBr(block_end);
    }

    set_current_block(block_end);
}

void FortranLLVM::visit(const Nodecl::ForStatement& node)
{
    TrackLocation loc(this, node.get_locus());
    Nodecl::NodeclBase loop_control = node.get_loop_header();
    Nodecl::NodeclBase body = node.get_statement();

    if (!loop_control.is<Nodecl::RangeLoopControl>())
        internal_error("Not yet implemented", 0);

    Nodecl::RangeLoopControl range_loop_control
        = loop_control.as<Nodecl::RangeLoopControl>();

    Nodecl::NodeclBase ind_var = range_loop_control.get_induction_variable();
    Nodecl::NodeclBase lower = range_loop_control.get_lower();
    Nodecl::NodeclBase upper = range_loop_control.get_upper();
    Nodecl::NodeclBase step = range_loop_control.get_step();

    bool constant_step = step.is_null() || step.is_constant();
    bool positive_step = constant_step && const_value_is_positive(step.get_constant());

    llvm::Value* vind_var = eval_expression(ind_var);

    llvm::Value* vstart = eval_expression(lower);
    llvm::Value* vend = eval_expression(upper);

    llvm::Value* vstep;
    if (step.is_null())
        vstep = get_integer_value(1, ind_var.get_symbol().get_type());
    else
        vstep = eval_expression(step);

    llvm::Value *vsign = nullptr;
    if (!constant_step)
    {   
        llvm::Value *vsign_check = ir_builder->CreateICmpSLT(
                vstep, get_integer_value(0, ind_var.get_symbol().get_type()));
        vsign = ir_builder->CreateSelect(vsign_check,
                get_integer_value(-1, ind_var.get_symbol().get_type()),
                get_integer_value(1, ind_var.get_symbol().get_type()));
    }


    ir_builder->CreateStore(vstart, vind_var);

    llvm::BasicBlock *block_check = llvm::BasicBlock::Create(
        llvm_context, "loop.check", get_current_function());
    llvm::BasicBlock *block_body = llvm::BasicBlock::Create(
        llvm_context, "loop.body", get_current_function());
    llvm::BasicBlock *block_end = llvm::BasicBlock::Create(
        llvm_context, "loop.end", get_current_function());

    ir_builder->CreateBr(block_check);

    set_current_block(block_check);
    llvm::Value* vcheck;
    if (constant_step)
    {
        if (positive_step)
            // i <= U
            vcheck = ir_builder->CreateICmpSLE(
                    ir_builder->CreateLoad(vind_var),
                    vend);
        else
            // i >= U
            vcheck = ir_builder->CreateICmpSGE(
                    ir_builder->CreateLoad(vind_var),
                    vend);
    }
    else
    {
        // I'm aware that replicating the loop could be more efficient but for now this will do
        // i * S <= U * S (i.e. if S < 0 then this is i * |S| >= U * |S|)
        // Here S will be the sign of the step (i.e. 1 or -1)
        vcheck = ir_builder->CreateICmpSLE(
                ir_builder->CreateMul(ir_builder->CreateLoad(vind_var), vsign),
                ir_builder->CreateMul(vend, vsign));
    }

    ir_builder->CreateCondBr(vcheck, block_body, block_end);

    set_current_block(block_body);
    walk(body);
    ir_builder->CreateStore(
        ir_builder->CreateAdd(ir_builder->CreateLoad(vind_var), vstep),
        vind_var);
    ir_builder->CreateBr(block_check);

    set_current_block(block_end);
}

void FortranLLVM::visit(const Nodecl::WhileStatement& node)
{
    TrackLocation loc(this, node.get_locus());
    llvm::BasicBlock *while_check = llvm::BasicBlock::Create(
        llvm_context, "while.check", get_current_function());
    llvm::BasicBlock *while_body = llvm::BasicBlock::Create(
        llvm_context, "while.body", get_current_function());
    llvm::BasicBlock *while_end = llvm::BasicBlock::Create(
        llvm_context, "while.end", get_current_function());

    ir_builder->CreateBr(while_check);

    set_current_block(while_check);
    llvm::Value *check = eval_expression(node.get_condition());
    ir_builder->CreateCondBr(
        ir_builder->CreateZExtOrTrunc(check, llvm_types.i1),
        while_body,
        while_end);

    set_current_block(while_body);
    walk(node.get_statement());

    ir_builder->CreateBr(while_check);

    set_current_block(while_end);
}

void FortranLLVM::visit(const Nodecl::SwitchStatement &node)
{
    FortranLLVM::TrackLocation loc(this, node);
    push_switch();

    current_switch().end_block = llvm::BasicBlock::Create(llvm_context, "switch.end", get_current_function());

    current_switch().expr = node.get_switch();
    current_switch().value = eval_expression(node.get_switch());

    walk(node.get_statement());

    if (!current_switch().default_case.is_null())
    {
        FortranLLVM::TrackLocation loc(this, current_switch().default_case);
        walk(current_switch().default_case.as<Nodecl::DefaultStatement>().get_statement());
    }

    ir_builder->CreateBr(current_switch().end_block);

    set_current_block(current_switch().end_block);

    pop_switch();
}

void FortranLLVM::visit(const Nodecl::DefaultStatement &node)
{
    current_switch().default_case = node;
}

void FortranLLVM::visit(const Nodecl::CaseStatement &node)
{
    FortranLLVM::TrackLocation loc(this, node);
    Nodecl::List cases = node.get_case().as<Nodecl::List>();
    
    TL::Type t = current_switch().expr.get_type().no_ref();

    llvm::BasicBlock *switch_case = llvm::BasicBlock::Create(llvm_context, "switch.case", get_current_function());

    for (Nodecl::NodeclBase current_case : cases)
    {
        if (current_case.is<Nodecl::Range>())
        {
            internal_error("Not yet implemented", 0);
        }
        else
        {
            llvm::Value *current_value = eval_expression(current_case);
            llvm::Value *check = nullptr;
            if (t.is_signed_integral() || t.is_bool())
            {
                check = ir_builder->CreateICmpEQ(current_switch().value, current_value);
            }
            else if (t.is_fortran_character())
            {
                // Logical type required by CharacterCompareEQ
                static Nodecl::NodeclBase logical_type;
                if (logical_type.is_null())
                    logical_type = Nodecl::Type::make(fortran_get_default_logical_type());
                FortranVisitorLLVMExpression::CharacterCompareEQ character_compare_eq(this, logical_type);
                check = character_compare_eq(
                            current_switch().expr, current_case,
                            current_switch().value, current_value);
                // FIXME: Our string comparator will return a LOGICAL (i8 at
                // least) but LLVM requires a bit (i1), so we need to truncate
                // it here.
                check = ir_builder->CreateZExtOrTrunc(check, llvm_types.i1);
            }
            else
            {
                internal_error("Unexpected type '%s'\n",
                        print_declarator(t.get_internal_type()));
            }

            llvm::BasicBlock *next_check = llvm::BasicBlock::Create(llvm_context, "switch.case.next_check", get_current_function());
            ir_builder->CreateCondBr(check, switch_case, next_check);
            set_current_block(next_check);
        }
    }

    llvm::BasicBlock *switch_case_end = llvm::BasicBlock::Create(llvm_context, "switch.case.end", get_current_function());
    ir_builder->CreateBr(switch_case_end);

    set_current_block(switch_case);
    walk(node.get_statement());
    ir_builder->CreateBr(current_switch().end_block);

    set_current_block(switch_case_end);
}

void FortranLLVM::visit(const Nodecl::FortranPrintStatement& node)
{
    FortranLLVM::TrackLocation loc(this, node);

    // TODO: Implement something fancier than list formating
    Nodecl::NodeclBase fmt = node.get_format();
    ERROR_CONDITION(!fmt.is<Nodecl::Text>()
                        || (fmt.as<Nodecl::Text>().get_text() != "*"),
                    "Only 'PRINT *' is implemented",
                    0);

    // Allocate data transfer structure
    llvm::Value *dt_parm
        = ir_builder->CreateAlloca(gfortran_rt.st_parameter_dt.get(), nullptr, "dt_parm");

    // dt_parm.common.filename = "file";
    // dt_parm.common.line = "file";
    // dt_parm.common.flags = 128;
    ir_builder->CreateStore(
            ir_builder->CreateGlobalStringPtr(node.get_filename()),
        gep_for_field(
            gfortran_rt.st_parameter_dt.get(), dt_parm, { "common", "filename" }));
    ir_builder->CreateStore(get_integer_value_32(node.get_line()),
                            gep_for_field(gfortran_rt.st_parameter_dt.get(),
                                          dt_parm,
                                          { "common", "line" }));
    ir_builder->CreateStore(get_integer_value_32(128),
                            gep_for_field(gfortran_rt.st_parameter_dt.get(),
                                          dt_parm,
                                          { "common", "flags" }));
    ir_builder->CreateStore(get_integer_value_32(6),
                            gep_for_field(gfortran_rt.st_parameter_dt.get(),
                                          dt_parm,
                                          { "common", "unit" }));

    ir_builder->CreateCall(gfortran_rt.st_write.get(), { dt_parm });

    Nodecl::List io_items = node.get_io_items().as<Nodecl::List>();
    // FIXME: Refactor
    for (Nodecl::NodeclBase n : io_items)
    {
        TL::Type t = n.get_type();
        if (fortran_is_character_type(t.get_internal_type()))
        {
            llvm::Value *expr = eval_expression(n);

            expr = ir_builder->CreatePointerCast(expr, llvm_types.ptr_i8);

            ir_builder->CreateCall(gfortran_rt.transfer_character_write.get(),
                                   { dt_parm, expr, eval_sizeof(n) });
        }
        else if (t.no_ref().is_signed_integral())
        {
            llvm::Value *expr = eval_expression_to_memory(n);

            expr = ir_builder->CreatePointerCast(expr, llvm_types.ptr_i8);

            ir_builder->CreateCall(gfortran_rt.transfer_integer_write.get(),
                                   { dt_parm, expr, eval_sizeof(n) });
        }
        else if (t.no_ref().is_bool())
        {
            llvm::Value *expr = eval_expression_to_memory(n);

            expr = ir_builder->CreatePointerCast(expr, llvm_types.ptr_i8);

            ir_builder->CreateCall(gfortran_rt.transfer_logical_write.get(),
                                   { dt_parm, expr, eval_sizeof(n) });
        }
        else if (t.no_ref().is_float()
                || t.no_ref().is_double())
        {
            llvm::Value *expr = eval_expression_to_memory(n);

            expr = ir_builder->CreatePointerCast(expr, llvm_types.ptr_i8);

            ir_builder->CreateCall(gfortran_rt.transfer_real_write.get(),
                                   { dt_parm, expr, eval_sizeof(n) });
        }
        else if (t.no_ref().is_complex())
        {
            llvm::Value *expr = eval_expression_to_memory(n);

            expr = ir_builder->CreatePointerCast(expr, llvm_types.ptr_i8);

            ir_builder->CreateCall(gfortran_rt.transfer_complex_write.get(),
                                   { dt_parm, expr, eval_sizeof(t.no_ref().complex_get_base_type()) });
        }
        else if (t.no_ref().is_array()
                 && !t.no_ref().array_requires_descriptor())
        {
            // Create a descriptor for this array
            llvm::Type *descriptor_type = get_gfortran_array_descriptor_type(t.no_ref());
            // Now create a temporary for it
            llvm::Value *descriptor_addr = ir_builder->CreateAlloca(descriptor_type, nullptr);

            // Get the address of the first element
            llvm::Value *base_address = ir_builder->CreatePointerCast(
                eval_expression(n), llvm_types.ptr_i8);

            // Fill the descriptor
            fill_descriptor_info(t.no_ref(), descriptor_addr, base_address);

            llvm::Value* expr = ir_builder->CreatePointerCast(descriptor_addr, llvm_types.ptr_i8);

            TL::Type base_type = t.no_ref().fortran_array_base_element();
            llvm::Value *kind = nullptr, *charlen = nullptr;
            if (base_type.is_fortran_character())
            {
                kind = get_integer_value_32(1);
                // FIXME - Does not work with CHARACTER(LEN=*)
                charlen = eval_length_of_character(base_type);
            }
            else
            {
                if (base_type.is_complex())
                    base_type = base_type.complex_get_base_type();
                kind = eval_sizeof(base_type);
                charlen = get_integer_value_32(0);
            }


            ir_builder->CreateCall(gfortran_rt.transfer_array_write.get(),
                                   { dt_parm, expr, kind, charlen });
        }
        else if (t.no_ref().is_array()
                 && t.no_ref().array_requires_descriptor())
        {
            llvm::Value* expr = eval_expression(n);
            // This should be the address of the descriptor already
            expr = ir_builder->CreatePointerCast(expr, llvm_types.ptr_i8);

            TL::Type base_type = t.no_ref().fortran_array_base_element();
            llvm::Value *kind = nullptr, *charlen = nullptr;
            if (base_type.is_fortran_character())
            {
                kind = get_integer_value_32(1);
                // FIXME - Does not work with CHARACTER(LEN=*)
                charlen = eval_length_of_character(base_type);
            }
            else
            {
                if (base_type.is_complex())
                    base_type = base_type.complex_get_base_type();
                kind = eval_sizeof(base_type);
                charlen = get_integer_value_32(0);
            }


            ir_builder->CreateCall(gfortran_rt.transfer_array_write.get(),
                                   { dt_parm, expr, kind, charlen });
        }
        else
        {
            internal_error("Type '%s' not yet implemented",
                           print_declarator(t.get_internal_type()));
        }
    }

    ir_builder->CreateCall(gfortran_rt.st_write_done.get(), { dt_parm });
}

void FortranLLVM::visit(const Nodecl::FortranAllocateStatement& node)
{
    // FIXME - We have to honour the options
    Nodecl::List items = node.get_items().as<Nodecl::List>();

    for (Nodecl::List::iterator alloc_it = items.begin();
         alloc_it != items.end();
         alloc_it++)
    {
        // The representation of this node (ab)uses expressions to represent
        // the allocation items, fortunately for us the subscripted item should
        // already be an ALLOCATABLE or POINTER array. Note that for the case
        // of the POINTER there is an extra indirection node that has to be
        // ignored.
        //
        // I think that upstream has a ticket about this representation being
        // unwieldy.
        if (alloc_it->is<Nodecl::ArraySubscript>())
        {
            Nodecl::ArraySubscript array_subscript
                = alloc_it->as<Nodecl::ArraySubscript>();
            allocate_array(array_subscript);
        }
        else
        {
            internal_error("Scalar allocate not implemented yet", 0);
        }
    }
}

void FortranLLVM::visit(const Nodecl::FortranDeallocateStatement& node)
{
    // FIXME - We have to honour the options
    Nodecl::List items = node.get_items().as<Nodecl::List>();

    for (Nodecl::List::iterator dealloc_it = items.begin();
         dealloc_it != items.end();
         dealloc_it++)
    {
        TrackLocation loc(this, *dealloc_it);

        Nodecl::NodeclBase dealloc_sym = *dealloc_it;
        ERROR_CONDITION(!dealloc_sym.is<Nodecl::Symbol>(), "Invalid node", 0);

        TL::Symbol symbol = dealloc_sym.as<Nodecl::Symbol>().get_symbol();

        if (symbol.is_allocatable())
        {
            // ALLOCATABLE
        }
        else if (symbol.get_type().is_pointer())
        {
            // POINTER
            internal_error("Not implemented yet", 0);
        }
        else
        {
            internal_error("Code unreachable", 0);
        }

        llvm::Value *descriptor_addr = get_value(symbol);

        llvm::Value *field_addr_base_addr
            = array_descriptor_addr_base_addr(descriptor_addr);

        llvm::BasicBlock *already_deallocated = llvm::BasicBlock::Create(
            llvm_context, "allocate.already_deallocated", get_current_function());
        llvm::BasicBlock *not_deallocated = llvm::BasicBlock::Create(
            llvm_context, "allocate.not_deallocated", get_current_function());

        ir_builder->CreateCondBr(
            ir_builder->CreateICmpEQ(
                ir_builder->CreatePtrToInt(
                    ir_builder->CreateLoad(field_addr_base_addr),
                    llvm_types.i64),
                get_integer_value_64(0)),
            already_deallocated,
            not_deallocated);

        set_current_block(already_deallocated);
        {
            std::stringstream ss;
            ss << "Array '" << symbol.get_name() << "' already deallocated";
            gfortran_runtime_error(dealloc_it->get_locus(), ss.str());
        }
        ir_builder->CreateBr(not_deallocated); // Appease IR checker

        set_current_block(not_deallocated);
        ir_builder->CreateCall(
            gfortran_rt.free.get(),
            { ir_builder->CreateLoad(field_addr_base_addr) });
        ir_builder->CreateStore(
            llvm::ConstantPointerNull::get(llvm_types.ptr_i8),
            field_addr_base_addr);
    }
}

}
