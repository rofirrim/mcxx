/*--------------------------------------------------------------------

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
#include "tl-compilerphase.hpp"
#include "fortran03-buildscope.h"
#include "fortran03-scope.h"
#include "fortran03-mangling.h"
#include "fortran03-exprtype.h"
#include "fortran03-typeutils.h"
#include "fortran03-cexpr.h"
#include "tl-compilerpipeline.hpp"
#include "tl-source.hpp"
#include "cxx-cexpr.h"
#include "cxx-entrylist.h"
#include "cxx-driver-utils.h"
#include "cxx-diagnostic.h"
#include "string_utils.h"
#include <ctype.h>

#include "cxx-lexer.h"
#include "cxx-typeenviron.h"

#include "llvm/Support/raw_os_ostream.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/Value.h"
#include "llvm/IR/Argument.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"

namespace Codegen
{
FortranLLVM::FortranLLVM()
{
    set_phase_name("Fortran LLVM codegen");
    set_phase_description(
        "This phase emits in LLVM IR the intermediate representation of the "
        "compiler");
}

void FortranLLVM::codegen(const Nodecl::NodeclBase &n, std::ostream *out)
{
    if (!is_file_output())
    {
        // Let FortranBase handle this case
        base.codegen(n, out);
    }
    else
    {
        file = out;
        walk(n);
    }
}

void FortranLLVM::codegen_cleanup()
{
}

namespace
{
void ensure_llvm_initialized()
{
    static bool init = false;
    if (!init)
    {
        llvm::InitializeAllTargetInfos();
        llvm::InitializeAllTargets();
        llvm::InitializeAllTargetMCs();
        init = true;
    }
}
}

void FortranLLVM::visit(const Nodecl::TopLevel &node)
{
    ensure_llvm_initialized();

    std::unique_ptr<llvm::Module> old_module;
    std::swap(old_module, current_module);

    current_module = llvm::make_unique<llvm::Module>(
        TL::CompilationProcess::get_current_file().get_filename(),
        llvm_context);

    std::string default_triple = llvm::sys::getDefaultTargetTriple();
    std::string error_message;
    const llvm::Target *target = llvm::TargetRegistry::lookupTarget(default_triple, error_message);
    if (target == NULL)
        fatal_error("Cannot get a LLVM target for triple '%s' due to '%s'\n", default_triple.c_str(), error_message.c_str());

    // Generic CPU, no special features or options
    llvm::TargetOptions target_options;
    llvm::Optional<llvm::Reloc::Model> reloc_model;
    llvm::TargetMachine *target_machine = target->createTargetMachine(
        default_triple, "generic", "", target_options, reloc_model);

    // Set triple and data layout of the target machine
    current_module->setTargetTriple(default_triple);
    current_module->setDataLayout(target_machine->createDataLayout());

    ir_builder = std::unique_ptr<llvm::IRBuilder<> >(
        new llvm::IRBuilder<>(llvm_context));

    initialize_llvm_context();

    walk(node.get_top_level());

    llvm::raw_os_ostream ros(*file);
    current_module->print(ros,
                          /* AssemblyAnnotationWriter */ nullptr);
    // current_module->dump();

    std::swap(old_module, current_module);
}

void FortranLLVM::visit(const Nodecl::FunctionCode &node)
{
    // Create a function with the proper type
    TL::Symbol sym = node.get_symbol();

    llvm::Type *function_type = nullptr;

    std::string mangled_name;
    if (sym.is_fortran_main_program())
    {
        mangled_name = "MAIN__";
        function_type = llvm::FunctionType::get(
            llvm_types.void_, {}, /* isVarArg */ false);
    }
    else
    {
        mangled_name = fortran_mangle_symbol(sym.get_internal_symbol());
        function_type = get_llvm_type(sym.get_type());
    }

    llvm::Constant *c = current_module->getOrInsertFunction(
        mangled_name,
        llvm::cast<llvm::FunctionType>(function_type),
        /* no attributes so far */ llvm::AttributeSet());

    llvm::Function *fun = llvm::cast<llvm::Function>(c);

    // Set argument names
    TL::ObjectList<TL::Symbol> related_symbols = sym.get_related_symbols();
    TL::ObjectList<TL::Symbol>::iterator related_symbols_it
        = related_symbols.begin();

    llvm::Function::ArgumentListType &llvm_fun_args = fun->getArgumentList();
    llvm::Function::ArgumentListType::iterator llvm_fun_args_it
        = llvm_fun_args.begin();

    clear_mappings();
    while (related_symbols_it != related_symbols.end()
           && llvm_fun_args_it != llvm_fun_args.end())
    {
        llvm::Value &v = *llvm_fun_args_it;
        TL::Symbol s = *related_symbols_it;

        v.setName(s.get_name());
        map_symbol_to_value(s, &v);

        related_symbols_it++;
        llvm_fun_args_it++;
    }
    ERROR_CONDITION(related_symbols_it != related_symbols.end()
                        || llvm_fun_args_it != llvm_fun_args.end(),
                    "Mismatch between TL and llvm::Arguments",
                    0);

    set_current_function(fun);
    llvm::BasicBlock *entry_basic_block
        = llvm::BasicBlock::Create(llvm_context, "entry", fun);
    set_current_block(entry_basic_block);
    push_allocating_block(entry_basic_block);

    walk(node.get_statements());

    if (sym.is_fortran_main_program()
            || sym.get_type().returns().is_void())
    {
        ir_builder->CreateRet(nullptr);
    }
    else
    {
        // Return the value in the return variable
        TL::Symbol return_variable = sym.get_result_variable();
        ERROR_CONDITION(!return_variable.is_valid(), "Result variable is missing?", 0);
        // Make sure it has been emitted
        emit_variable(return_variable);
        llvm::Value* addr_ret_val = get_value(return_variable);
        llvm::Value* value_ret_val = ir_builder->CreateLoad(addr_ret_val);
        ir_builder->CreateRet(value_ret_val);
    }

    pop_allocating_block();
    clear_current_function();

    if (sym.is_fortran_main_program())
    {
        emit_main(fun);
    }
}

void FortranLLVM::visit(const Nodecl::Context &node)
{
    walk(node.get_in_context());
}

void FortranLLVM::visit(const Nodecl::CompoundStatement &node)
{
    walk(node.get_statements());
}

void FortranLLVM::visit(const Nodecl::ExpressionStatement &node)
{
    eval_expression(node.get_nest());
}

void FortranLLVM::visit(const Nodecl::ObjectInit& node)
{
    
    TL::Symbol sym = node.get_symbol();

    if (sym.is_variable()
            && sym.is_saved_expression())
    {
        // Model this like an assignment to the saved expression
        emit_variable(sym);
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

    if (!step.is_null() && !step.is_constant())
        internal_error("Not yet implemented", 0);

    if (const_value_is_positive(step.get_constant()))
    {
    }
    else
        internal_error("Not yet implemented", 0);

    llvm::Value* vind_var = eval_expression(ind_var);
    llvm::Value* vlower = eval_expression(lower);
    llvm::Value* vupper = eval_expression(upper);
    llvm::Value* vstep;
    if (step.is_null())
        vstep = get_integer_value(1, ind_var.get_symbol().get_type());
    else
        vstep = eval_expression(step);

    ir_builder->CreateStore(vlower, vind_var);

    llvm::BasicBlock *block_check = llvm::BasicBlock::Create(
        llvm_context, "loop.check", get_current_function());
    llvm::BasicBlock *block_body = llvm::BasicBlock::Create(
        llvm_context, "loop.body", get_current_function());
    llvm::BasicBlock *block_end = llvm::BasicBlock::Create(
        llvm_context, "loop.end", get_current_function());

    ir_builder->CreateBr(block_check);

    set_current_block(block_check);
    llvm::Value* vcheck = ir_builder->CreateICmpSLE(
            ir_builder->CreateLoad(vind_var),
            vupper);
    ir_builder->CreateCondBr(vcheck, block_body, block_end);

    set_current_block(block_body);
    walk(body);
    ir_builder->CreateStore(
        ir_builder->CreateAdd(ir_builder->CreateLoad(vind_var), vstep),
        vind_var);
    ir_builder->CreateBr(block_check);

    set_current_block(block_end);
}

class FortranVisitorLLVMExpressionBase : public Nodecl::NodeclVisitor<void>
{
  protected:
    FortranLLVM *llvm_visitor;
    llvm::Value *value = nullptr;

  public:
    FortranVisitorLLVMExpressionBase(FortranLLVM *llvm_visitor)
        : llvm_visitor(llvm_visitor)
    {
    }

    llvm::Value *get_value()
    {
        ERROR_CONDITION(value == NULL, "Invalid value gathered", 0);
        return value;
    }

    virtual void unhandled_node(const Nodecl::NodeclBase &n)
    {
        internal_error("Unexpected node '%s'\n",
                       ast_print_node_type(n.get_kind()))
    }
};

class FortranVisitorLLVMExpression : public FortranVisitorLLVMExpressionBase
{
  public:
    using FortranVisitorLLVMExpressionBase::FortranVisitorLLVMExpressionBase;

    void visit(const Nodecl::Symbol &node)
    {
        llvm_visitor->emit_variable(node.get_symbol());
        value = llvm_visitor->get_value(node.get_symbol());
    }

    llvm::Value *scalar_conversion(TL::Type dest, TL::Type orig, llvm::Value* value_nest)
    {
        ERROR_CONDITION(dest.no_ref().is_array(), "Invalid type", 0);
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

    void visit(const Nodecl::Conversion& node)
    {
        TL::Type dest = node.get_type();
        TL::Type orig = node.get_nest().get_type();

        llvm::Value *nest_value = llvm_visitor->eval_expression(node.get_nest());

        if (dest.is_array()
                && orig.no_ref().is_array())
        {
            // We do not represent values of array, so we let the address pass-through
            // FIXME: Arrays with region that are demoted to non-region arrays
            ERROR_CONDITION(
                !dest.array_element().is_same_type(orig.no_ref().array_element()),
                "array-wise value conversions not implemented yet",
                0);
            value = nest_value;
        }
        else if (dest.is_array()
                && !orig.no_ref().is_array())
        {
            value = scalar_conversion(dest.array_base_element(), orig, nest_value);
        }
        else
        {
            value = scalar_conversion(dest, orig, nest_value);
        }
    }

    bool is_scalar_to_array(const Nodecl::NodeclBase &n)
    {
        return n.get_type().is_array()
               && !n.no_conv().get_type().no_ref().is_array();
    }

    void visit(const Nodecl::Assignment& node)
    {
        llvm::Value *vlhs = llvm_visitor->eval_expression(node.get_lhs());
        llvm::Value *vrhs = llvm_visitor->eval_expression(node.get_rhs());

        TL::Type lhs_type = node.get_lhs().get_type();
        TL::Type rhs_type = node.get_rhs().get_type();

        if (lhs_type.no_ref().is_array() && rhs_type.is_array())
        {
            lhs_type = lhs_type.no_ref();
            if (!lhs_type.array_is_vla()
                && !lhs_type.array_is_region()
                && !lhs_type.array_requires_descriptor()
                && !rhs_type.array_is_vla()
                && !rhs_type.array_is_region()
                && !rhs_type.array_requires_descriptor())
            {
                // If both are statically sized arrays we use LLVM IR
                // feature of array-wise operations
                value = llvm_visitor->ir_builder->CreateStore(vrhs, vlhs);
            }
            else
            {
                ERROR_CONDITION(lhs_type.array_is_region()
                                    || rhs_type.array_is_region(),
                                "Regions not implemented yet",
                                0);

                // Do a loop of stores
                llvm::Value *idx_var = llvm_visitor->ir_builder->CreateAlloca(
                    llvm_visitor->llvm_types.i64, nullptr, "array_op_idx.addr");
                llvm::Value *array_size
                    = llvm_visitor->evaluate_size_of_array(lhs_type);
                create_array_op_loop(
                    idx_var,
                    array_size,
                    [&, this]()
                    {
                        llvm::Value *idx_value
                            = llvm_visitor->ir_builder->CreateLoad(
                                idx_var, "array_op_idx.val");
                        llvm::Value *lhs_elem_addr
                            = compute_addr_from_linear_element(
                                lhs_type, vlhs, idx_value);

                        if (is_scalar_to_array(node.get_rhs()))
                        {
                            llvm_visitor->ir_builder->CreateStore(
                                vrhs, lhs_elem_addr);
                        }
                        else
                        {
                            llvm::Value *rhs_elem_addr
                                = compute_addr_from_linear_element(
                                    rhs_type, vrhs, idx_value);
                            llvm::Value *rhs_elem_val
                                = llvm_visitor->ir_builder->CreateLoad(
                                    rhs_elem_addr);

                            llvm_visitor->ir_builder->CreateStore(
                                rhs_elem_val, lhs_elem_addr);
                        }
                    });

                value = vlhs;
            }
        }
        else if (!lhs_type.no_ref().is_array() && !rhs_type.is_array())
        {
            value = llvm_visitor->ir_builder->CreateStore(vrhs, vlhs);
        }
        else
        {
            internal_error("Unexpected assignment with lhs=%s and rhs=%s\n",
                    print_declarator(lhs_type.get_internal_type()),
                    print_declarator(rhs_type.get_internal_type()));
        }
    }


    // void visit(const Nodecl::StructuredValue& node);
    void visit(const Nodecl::BooleanLiteral& node)
    {
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

    // void visit(const Nodecl::ComplexLiteral& node);

    void visit(const Nodecl::ObjectInit& node)
    {
        std::cerr << "Unhandled object init. Symbol = " << node.get_symbol().get_name() << " " << symbol_kind_name(node.get_symbol().get_internal_symbol()) << std::endl;
    }

    void visit(const Nodecl::Neg &node)
    {
        Nodecl::NodeclBase rhs = node.get_rhs();
        llvm::Value *vrhs = llvm_visitor->eval_expression(rhs);

        // There is no neg instruction. So "neg x" is represented as "sub 0, x"
        if (rhs.get_type().is_signed_integral())
        {
            llvm::Value *z = llvm_visitor->get_integer_value(0, rhs.get_type());
            value = llvm_visitor->ir_builder->CreateSub(z, vrhs);
        }
        else if (rhs.get_type().is_floating_type())
        {
            llvm::Value *z = llvm::ConstantFP::get(llvm_visitor->llvm_context,
                                                   llvm::APFloat(0.0));
            value = llvm_visitor->ir_builder->CreateFSub(z, vrhs);
        }
        else
        {
            internal_error(
                "Code unreachable for unary operator %s. Type is '%s'",
                ast_print_node_type(node.get_kind()),
                print_declarator(rhs.get_type().get_internal_type()));
        }
    }

    void visit(const Nodecl::Plus &node)
    {
        // No-operation
        walk(node.get_rhs());
    }

    void visit(const Nodecl::LogicalAnd& node)
    {
        Nodecl::NodeclBase lhs = node.get_lhs();
        Nodecl::NodeclBase rhs = node.get_rhs();
        llvm::Value *vlhs = llvm_visitor->eval_expression(lhs);
        llvm::BasicBlock *block_eval_lhs = llvm_visitor->get_current_block();

        llvm::Value *vcond = llvm_visitor->ir_builder->CreateZExtOrTrunc(
            vlhs, llvm_visitor->llvm_types.i1);

        llvm::BasicBlock *block_eval_rhs = llvm::BasicBlock::Create(
            llvm_visitor->llvm_context, "and.rhs", llvm_visitor->get_current_function());
        llvm::BasicBlock *block_end = llvm::BasicBlock::Create(
            llvm_visitor->llvm_context, "and.end", llvm_visitor->get_current_function());
        llvm_visitor->ir_builder->CreateCondBr(vcond, block_eval_rhs, block_end);

        llvm_visitor->set_current_block(block_eval_rhs);
        llvm::Value *vrhs = llvm_visitor->eval_expression(rhs);
        llvm_visitor->ir_builder->CreateBr(block_end);

        llvm_visitor->set_current_block(block_end);
        value = llvm_visitor->ir_builder->CreatePHI(
            llvm_visitor->get_llvm_type(node.get_type()), 2);
        llvm::cast<llvm::PHINode>(value)->addIncoming(vlhs, block_eval_lhs);
        llvm::cast<llvm::PHINode>(value)->addIncoming(vrhs, block_eval_rhs);
    }

    void visit(const Nodecl::LogicalOr& node)
    {
        Nodecl::NodeclBase lhs = node.get_lhs();
        Nodecl::NodeclBase rhs = node.get_rhs();
        llvm::BasicBlock *block_eval_lhs = llvm_visitor->get_current_block();
        llvm::Value *vlhs = llvm_visitor->eval_expression(lhs);

        llvm::Value *vcond = llvm_visitor->ir_builder->CreateZExtOrTrunc(
            vlhs, llvm_visitor->llvm_types.i1);

        llvm::BasicBlock *block_eval_rhs = llvm::BasicBlock::Create(
            llvm_visitor->llvm_context, "or.rhs", llvm_visitor->get_current_function());
        llvm::BasicBlock *block_end = llvm::BasicBlock::Create(
            llvm_visitor->llvm_context, "or.end", llvm_visitor->get_current_function());
        llvm_visitor->ir_builder->CreateCondBr(vcond, block_end, block_eval_rhs);

        llvm_visitor->set_current_block(block_eval_rhs);
        llvm::Value *vrhs = llvm_visitor->eval_expression(rhs);
        llvm_visitor->ir_builder->CreateBr(block_end);

        llvm_visitor->set_current_block(block_end);
        value = llvm_visitor->ir_builder->CreatePHI(
            llvm_visitor->get_llvm_type(node.get_type()), 2);
        llvm::cast<llvm::PHINode>(value)->addIncoming(vlhs, block_eval_lhs);
        llvm::cast<llvm::PHINode>(value)->addIncoming(vrhs, block_eval_rhs);
    }

    void visit(const Nodecl::LogicalNot& node)
    {
        Nodecl::NodeclBase rhs = node.get_rhs();
        TL::Type rhs_type = rhs.get_type();

        value = llvm_visitor->eval_expression(rhs);

        // select x, 1, 0
        value = llvm_visitor->ir_builder->CreateSelect(
            llvm_visitor->ir_builder->CreateZExtOrTrunc(
                value, llvm_visitor->llvm_types.i1),
            llvm_visitor->get_integer_value(1, rhs_type),
            llvm_visitor->get_integer_value(0, rhs_type));
    }

    template <typename CreateSInt, typename CreateFloat>
    llvm::Value *arithmetic_binary_op_elemental_intrinsic(
        TL::Type lhs_type,
        TL::Type rhs_type,
        llvm::Value *lhs_val,
        llvm::Value *rhs_val,
        CreateSInt create_sint,
        CreateFloat create_float)
    {
        if (lhs_type.is_signed_integral() && rhs_type.is_signed_integral())
        {
            return create_sint(lhs_val, rhs_val);
        }
        else if (lhs_type.is_floating_type() && rhs_type.is_floating_type())
        {
            return create_float(lhs_val, rhs_val);
        }
        else
        {
            internal_error(
                "Code unreachable for arithmetic binary operator. Types are "
                "'%s' and "
                "'%s'",
                print_declarator(lhs_type.get_internal_type()),
                print_declarator(rhs_type.get_internal_type()));
        }
    }

    llvm::Value *compute_addr_from_linear_element(TL::Type t, llvm::Value* addr, llvm::Value* idx_value)
    {
        ERROR_CONDITION(!t.is_array(), "Invalid type", 0);

        TL::Type element_type = t.array_base_element();

        if (t.array_requires_descriptor())
        {
            internal_error("Not yet implemented", 0);
        }
        else if (t.array_is_region())
        {
            // FIXME - This could be contiguous
            internal_error("Not yet implemented", 0);
        }
        else
        {
            // Contiguous array
            llvm::Value *base_addr = llvm_visitor->ir_builder->CreatePtrToInt(
                addr, llvm_visitor->llvm_types.i64);

            llvm::Value *offset = llvm_visitor->ir_builder->CreateMul(
                idx_value, llvm_visitor->eval_sizeof_64(element_type));

            llvm::Value *addr
                = llvm_visitor->ir_builder->CreateAdd(base_addr, offset);

            return llvm_visitor->ir_builder->CreateIntToPtr(
                addr,
                llvm::PointerType::get(
                    llvm_visitor->get_llvm_type(element_type),
                    /* AddressSpace */ 0));
        }
    }

    template <typename EmitLoopBody>
    void create_array_op_loop(llvm::Value* idx_var, llvm::Value* upper, EmitLoopBody emit_loop_body)
    {
        llvm::BasicBlock *block_check
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "array_op.check",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *block_body
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "array_op.body",
                                       llvm_visitor->get_current_function());
        llvm::BasicBlock *block_end
            = llvm::BasicBlock::Create(llvm_visitor->llvm_context,
                                       "array_op.end",
                                       llvm_visitor->get_current_function());

        llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0), idx_var);

        llvm_visitor->ir_builder->CreateBr(block_check);
        llvm_visitor->set_current_block(block_check);

        llvm::Value *vcheck = llvm_visitor->ir_builder->CreateICmpSLT(
            llvm_visitor->ir_builder->CreateLoad(idx_var),
            upper);
        llvm_visitor->ir_builder->CreateCondBr(vcheck, block_body, block_end);

        llvm_visitor->set_current_block(block_body);

        emit_loop_body();

        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->ir_builder->CreateAdd(
                llvm_visitor->ir_builder->CreateLoad(idx_var),
                llvm_visitor->get_integer_value_64(1)),
            idx_var);
        llvm_visitor->ir_builder->CreateBr(block_check);

        llvm_visitor->set_current_block(block_end);
    }


    template <typename CreateSInt, typename CreateFloat>
    void arithmetic_binary_operator_array(const Nodecl::NodeclBase &lhs,
                                          const Nodecl::NodeclBase &rhs,
                                          CreateSInt create_sint,
                                          CreateFloat create_float)
    {
        TL::Type lhs_type = lhs.get_type();
        TL::Type rhs_type = rhs.get_type();

        TL::Type lhs_element_type = lhs_type.array_base_element();
        TL::Type rhs_element_type = rhs_type.array_base_element();

        ERROR_CONDITION(!lhs_element_type.is_same_type(rhs_element_type),
                        "Not implemented yet",
                        0);

        // TODO: Select easiest of the two to compute
        llvm::Value *array_size
            = llvm_visitor->evaluate_size_of_array(lhs_type);

        // Evaluate addresses
        llvm::Value *lhs_addr = llvm_visitor->eval_expression(lhs);
        llvm::Value *rhs_addr = llvm_visitor->eval_expression(rhs);

        // Allocate space for the result
        llvm::Value *result_addr = llvm_visitor->ir_builder->CreateAlloca(
            llvm_visitor->get_llvm_type(lhs_element_type), array_size);

        // Now create a loop
        llvm::Value *idx_var = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64, nullptr, "array_op_idx.addr");

        create_array_op_loop(
            idx_var,
            array_size,
            [&, this]()
            {
                llvm::Value *idx_value = llvm_visitor->ir_builder->CreateLoad(
                    idx_var, "array_op_idx.val");

                llvm::Value *lhs_elem_val;
                if (is_scalar_to_array(lhs))
                {
                    lhs_elem_val = lhs_addr;
                }
                else
                {
                    llvm::Value *lhs_elem_addr
                        = compute_addr_from_linear_element(
                            lhs_type, lhs_addr, idx_value);
                    lhs_elem_val
                        = llvm_visitor->ir_builder->CreateLoad(lhs_elem_addr);
                }

                llvm::Value *rhs_elem_val;
                if (is_scalar_to_array(rhs))
                {
                    rhs_elem_val = rhs_addr;
                }
                else
                {
                    llvm::Value *rhs_elem_addr
                        = compute_addr_from_linear_element(
                            rhs_type, rhs_addr, idx_value);
                    rhs_elem_val
                        = llvm_visitor->ir_builder->CreateLoad(rhs_elem_addr);
                }

                llvm::Value *result_value
                    = arithmetic_binary_op_elemental_intrinsic(lhs_element_type,
                                                               rhs_element_type,
                                                               lhs_elem_val,
                                                               rhs_elem_val,
                                                               create_sint,
                                                               create_float);

                llvm::Value *result_elem_addr
                    = compute_addr_from_linear_element(
                        lhs_type, result_addr, idx_value);
                llvm_visitor->ir_builder->CreateStore(result_value,
                                                      result_elem_addr);
            });

        value = result_addr;
    }

    template <typename Node, typename CreateSInt, typename CreateFloat>
    void arithmetic_binary_operator_array_scalar(const Node lhs,
                                                 const Node rhs,
                                                 CreateSInt create_sint,
                                                 CreateFloat create_float)
    {
        internal_error("Array + scalar not implemented yet", 0);
    }

    template <typename Node, typename CreateSInt, typename CreateFloat>
    void arithmetic_binary_operator(const Node node,
                               CreateSInt create_sint,
                               CreateFloat create_float)
    {
        Nodecl::NodeclBase lhs = node.get_lhs();
        Nodecl::NodeclBase rhs = node.get_rhs();

        TL::Type lhs_type = lhs.get_type();
        TL::Type rhs_type = rhs.get_type();

        if (lhs_type.is_array() || rhs_type.is_array())
        {
            if (lhs_type.is_array() == rhs_type.is_array())
            {
                if (!lhs_type.array_is_vla()
                    && !lhs_type.array_requires_descriptor()
                    && !rhs_type.array_is_vla()
                    && !rhs_type.array_requires_descriptor())
                {
                    // If both are statically sized arrays we use LLVM IR
                    // feature of array-wise operations
                    lhs_type = lhs_type.array_base_element();
                    rhs_type = rhs_type.array_base_element();
                    // FALL-THROUGH
                }
                else
                {
                    return arithmetic_binary_operator_array(
                        lhs, rhs, create_sint, create_float);
                }
            }
            else
            {
                internal_error("Code unreachable. Inconsistent types in operators", 0);
            }
        }

        llvm::Value *vlhs = llvm_visitor->eval_expression(lhs);
        llvm::Value *vrhs = llvm_visitor->eval_expression(rhs);

        value = arithmetic_binary_op_elemental_intrinsic(
            lhs_type, rhs_type, vlhs, vrhs, create_sint, create_float);
    }

#define CREATOR(Creator) [&, this](llvm::Value* lhs, llvm::Value* rhs) { return llvm_visitor->ir_builder->Creator(lhs, rhs); }
    void visit(const Nodecl::Add& node)
    {
        arithmetic_binary_operator(node,
                              CREATOR(CreateAdd),
                              CREATOR(CreateFAdd));
    }

    void visit(const Nodecl::Minus& node)
    {
        arithmetic_binary_operator(node,
                              CREATOR(CreateSub),
                              CREATOR(CreateFSub));
    }

    void visit(const Nodecl::Mul& node)
    {
        arithmetic_binary_operator(node,
                              CREATOR(CreateMul),
                              CREATOR(CreateFMul));
    }

    void visit(const Nodecl::Div& node)
    {
        arithmetic_binary_operator(node,
                              CREATOR(CreateSDiv),
                              CREATOR(CreateFDiv));
    }

    // void visit(const Nodecl::Power& node);

    template <typename Node, typename CreateSInt, typename CreateFloat>
    void arithmetic_binary_comparison(const Node node,
                                      CreateSInt create_sint,
                                      CreateFloat create_float)
    {
        arithmetic_binary_operator(node, create_sint, create_float);

        // Make sure the logical value stays in the proper integer size
        // and not just i1.
        // Note the usage of CreateZExtOrTrunc here instead of the usual SExt
        // otherwise the resulting value would be 0 or -1 and we want 0 or 1
        value = llvm_visitor->ir_builder->CreateZExtOrTrunc(
            value, llvm_visitor->get_llvm_type(node.get_type()));
    }

    void visit(const Nodecl::LowerThan &node)
    {
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpSLT),
                              CREATOR(CreateFCmpOLT));
    }

    void visit(const Nodecl::LowerOrEqualThan &node)
    {
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpSLE),
                              CREATOR(CreateFCmpOLE));
    }

    void visit(const Nodecl::GreaterThan &node)
    {
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpSGT),
                              CREATOR(CreateFCmpOGT));
    }

    void visit(const Nodecl::GreaterOrEqualThan &node)
    {
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpSGE),
                              CREATOR(CreateFCmpOGE));
    }

    void visit(const Nodecl::Equal &node)
    {
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpEQ),
                              CREATOR(CreateFCmpOEQ));
    }

    void visit(const Nodecl::Different &node)
    {
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpNE),
                              CREATOR(CreateFCmpONE));
    }
#undef CREATOR

    // void visit(const Nodecl::Concat& node);
    // void visit(const Nodecl::ClassMemberAccess& node);

    void visit(const Nodecl::Range& node)
    {
        // Do nothing
    }

    void visit(const Nodecl::StringLiteral& node)
    {
        std::string str = node.get_text();
        str = str.substr(1, str.size() - 2);
        value = llvm_visitor->ir_builder->CreateGlobalStringPtr(str);
    }

    void visit(const Nodecl::IntegerLiteral &node)
    {
        value = llvm_visitor->get_integer_value(
            // FIXME: INTEGER(16)
            const_value_cast_to_8(node.get_constant()),
            node.get_type());
    }

    void visit(const Nodecl::FloatingLiteral& node)
    {
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
    void visit(const Nodecl::ParenthesizedExpression& node)
    {
        // FIXME: Technically Fortran says that the operands are to be
        // evaluated in the program order if they are inside a parenthesis. For
        // now I do not know how to force such a thing in LLVM, so temporarily
        // we will assume this is a no-op
        walk(node.get_nest());
    }

    void visit(const Nodecl::ArraySubscript& node)
    {
        Nodecl::NodeclBase subscripted = node.get_subscripted();
        TL::Type subscripted_type = subscripted.get_type();
        TL::Type subscripted_type_noref = subscripted_type.no_ref();

        ERROR_CONDITION(
            !subscripted_type_noref.is_array(), "Expecting an array here", 0);

        if (subscripted_type_noref.array_requires_descriptor())
        {
            internal_error("Not yet implemented", 0);
        }
        else
        {
            bool is_vla = subscripted_type_noref.array_is_vla();

            Nodecl::List subscripts = node.get_subscripts().as<Nodecl::List>();
            std::vector<llvm::Value *> offset_list, size_list;
            if (is_vla)
            {
                offset_list.reserve(subscripts.size());
                size_list.reserve(subscripts.size());
            }
            else
            {
                offset_list.reserve(subscripts.size() + 1);
                offset_list.push_back(llvm_visitor->get_integer_value_32(0));
            }

            TL::Type current_array_type = subscripted_type_noref;
            for (Nodecl::NodeclBase index : subscripts)
            {
                Nodecl::NodeclBase lower, upper;
                current_array_type.array_get_bounds(lower, upper);
                llvm::Value *val_lower
                    = llvm_visitor->ir_builder->CreateSExtOrTrunc(
                        llvm_visitor->eval_expression(lower),
                        llvm_visitor->llvm_types.i64);
                llvm::Value *val_upper
                    = llvm_visitor->ir_builder->CreateSExtOrTrunc(
                        llvm_visitor->eval_expression(upper),
                        llvm_visitor->llvm_types.i64);

                llvm::Value *val_idx
                    = llvm_visitor->ir_builder->CreateSExtOrTrunc(
                        llvm_visitor->eval_expression(index),
                        llvm_visitor->llvm_types.i64);

                llvm::Value *val_offset
                    = llvm_visitor->ir_builder->CreateSub(val_idx, val_lower);
                offset_list.push_back(val_offset);

                if (is_vla)
                {
                    llvm::Value *val_size = llvm_visitor->ir_builder->CreateAdd(
                        llvm_visitor->ir_builder->CreateSub(val_upper,
                                                            val_lower),
                        llvm_visitor->get_integer_value_64(1));
                    size_list.push_back(val_size);
                }

                current_array_type = current_array_type.array_element();
            }

            llvm::Value *subscripted_val
                = llvm_visitor->eval_expression(subscripted);
            if (!is_vla)
            {
                // For constant sized arrays use a GEP
                value = llvm_visitor->ir_builder->CreateGEP(subscripted_val,
                                                            offset_list);
            }
            else
            {
                // Otherwise just compute an offset in elements using Horner's rule.
                std::vector<llvm::Value *>::iterator it_offsets = offset_list.begin();
                std::vector<llvm::Value *>::iterator it_sizes = size_list.begin();

                llvm::Value* val_addr = *it_offsets;
                it_offsets++;
                it_sizes++;

                while (it_offsets != offset_list.end()
                       && it_sizes != size_list.end())
                {
                    val_addr = llvm_visitor->ir_builder->CreateAdd(
                        *it_offsets,
                        llvm_visitor->ir_builder->CreateMul(*it_sizes, val_addr));

                    it_offsets++;
                    it_sizes++;
                }
                ERROR_CONDITION(it_offsets != offset_list.end()
                                    || it_sizes != size_list.end(),
                                "Lists do not match", 0);

                // Now multiply by the size of the type to get an offset in bytes
                ERROR_CONDITION(current_array_type.is_array(), "Should not be an array here", 0);
                val_addr = llvm_visitor->ir_builder->CreateMul(val_addr, llvm_visitor->eval_sizeof_64(current_array_type));

                // And add this offset in bytes to the base
                val_addr = llvm_visitor->ir_builder->CreateAdd(
                    llvm_visitor->ir_builder->CreatePtrToInt(
                        subscripted_val, llvm_visitor->llvm_types.i64),
                    val_addr);

                value = llvm_visitor->ir_builder->CreateIntToPtr(val_addr, subscripted_val->getType());
            }
        }
    }

    void visit(const Nodecl::FunctionCall& node)
    {
        Nodecl::NodeclBase called = node.get_called();
        Nodecl::List arguments = node.get_arguments().as<Nodecl::List>();

        ERROR_CONDITION(
            !called.is<Nodecl::Symbol>(), "We can only call functions", 0);
        TL::Symbol called_sym = called.get_symbol();

        if (called_sym.is_builtin())
            internal_error("Builtin calls not yet implemented", 0);

        TL::Type called_type = called_sym.get_type();
        ERROR_CONDITION(
            !called_type.is_function(), "Expecting a function type here", 0);

        if (called_type.lacks_prototype())
            internal_error(
                "Calls to unprototyped functions not yet implemented", 0);

        std::string mangled_name
            = fortran_mangle_symbol(called_sym.get_internal_symbol());
        llvm::FunctionType *function_type = llvm::cast<llvm::FunctionType>(
            llvm_visitor->get_llvm_type(called_sym.get_type()));

        llvm::Constant *c = llvm_visitor->current_module->getOrInsertFunction(
            mangled_name,
            function_type,
            /* no attributes so far */ llvm::AttributeSet());
        llvm::Function *fun = llvm::cast<llvm::Function>(c);

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

            llvm::Value *varg = llvm_visitor->eval_expression(arg);

            if (it_param->no_ref().is_array())
            {
                TL::Type array_type = it_param->no_ref();
                if (array_type.array_is_vla())
                {
                    TL::Type element_type = array_type.array_base_element();

                    // Cast to a pointer of the element
                    varg = llvm_visitor->ir_builder->CreateBitCast(
                        varg,
                        llvm::PointerType::get(
                            llvm_visitor->get_llvm_type(element_type),
                            /* AddressSpace */ 0));
                }
                else if (array_type.array_requires_descriptor())
                {
                    internal_error("Not yet implemented", 0);
                }
                else
                {
                    // Nothing special has to be done
                }
            }


            val_arguments.push_back(varg);

            it_arg++;
            it_param++;
        }
        ERROR_CONDITION(it_arg != arguments.end()
                            || it_param != parameters.end(),
                        "Mismatch between arguments and parameters",
                        0);

        value = llvm_visitor->ir_builder->CreateCall(fun, val_arguments);
    }

    // void visit(const Nodecl::FortranActualArgument& node);
    // void visit(const Nodecl::FortranIoSpec& node);
    // void visit(const Nodecl::FortranImpliedDo& node);
    // void visit(const Nodecl::FortranData& node);
    // void visit(const Nodecl::FortranEquivalence& node);
    // void visit(const Nodecl::FortranAlternateReturnArgument& node);
    // void visit(const Nodecl::FortranForall& node);
    // void visit(const Nodecl::FortranWhere& node);
    // void visit(const Nodecl::FortranBozLiteral& node);
    // void visit(const Nodecl::FortranHollerith& node);
    // void visit(const Nodecl::FortranUse& node);
    // void visit(const Nodecl::FortranUseOnly& node);
    // void visit(const Nodecl::FieldDesignator& node);
    // void visit(const Nodecl::IndexDesignator& node);
    // void visit(const Nodecl::UnknownPragma& node);
    // void visit(const Nodecl::PragmaCustomDeclaration& node);
    // void visit(const Nodecl::PragmaCustomClause& node);
    // void visit(const Nodecl::PragmaCustomLine& node);
    // void visit(const Nodecl::PragmaCustomDirective& node);
    // void visit(const Nodecl::PragmaClauseArg& node);
    // void visit(const Nodecl::SourceComment& node);
    // void visit(const Nodecl::Sizeof& node);
    // void visit(const Nodecl::Alignof& node);

    // void visit(const Nodecl::MulAssignment & node);
    // void visit(const Nodecl::DivAssignment & node);
    // void visit(const Nodecl::AddAssignment & node);
    // void visit(const Nodecl::MinusAssignment & node);

    // void visit(const Nodecl::Preincrement& node);
    // void visit(const Nodecl::Postincrement& node);
    // void visit(const Nodecl::Predecrement& node);
    // void visit(const Nodecl::Postdecrement& node);
};


llvm::Value *FortranLLVM::eval_expression(Nodecl::NodeclBase n)
{
    FortranVisitorLLVMExpression v(this);
    v.walk(n);
    return v.get_value();
}

llvm::IRBuilderBase::InsertPoint FortranLLVM::change_to_allocating_block()
{
    llvm::IRBuilderBase::InsertPoint saved_ip = ir_builder->saveIP();

    llvm::BasicBlock *alloca_bb = allocating_block();

    // FIXME: I'm pretty sure it is possible to do this better.
    llvm::BasicBlock::iterator bb_it = alloca_bb->begin();
    while (bb_it != alloca_bb->end()
            && !bb_it->isTerminator())
        bb_it++;

    ir_builder->SetInsertPoint(alloca_bb, bb_it);

    return saved_ip;
}

void FortranLLVM::return_from_allocating_block(llvm::IRBuilderBase::InsertPoint previous_ip)
{
    ir_builder->restoreIP(previous_ip);
}

llvm::Value *FortranLLVM::create_alloca(llvm::Type *t,
                                   llvm::Value *array_size,
                                   const llvm::Twine &name)
{
    llvm::IRBuilderBase::InsertPoint saved_ip = change_to_allocating_block();

    llvm::Value *tmp
        = ir_builder->CreateAlloca(t, array_size, name);

    return_from_allocating_block(saved_ip);

    return tmp;
}

llvm::Value *FortranLLVM::make_temporary(llvm::Value *v)
{
    llvm::Value *tmp = create_alloca(v->getType());
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
    // TODO: Eventually this will become more complicated
    return get_integer_value_64(t.no_ref().get_size());
}


llvm::Type *FortranLLVM::get_llvm_type(TL::Type t)
{
    if (t.is_lvalue_reference())
    {
        return llvm::PointerType::get(get_llvm_type(t.references_to()),
                                      /* AddressSpace */ 0);
    }
    // else if (t.is_pointer())
    // {
    //     return PointerType::get(
    //             get_llvm_type(t.points_to()),
    //             /* AddressSpace */ 0);
    // }
    else if (t.is_function())
    {
        TL::ObjectList<TL::Type> params = t.parameters();
        std::vector<llvm::Type *> llvm_params;
        llvm_params.reserve(params.size());

        for (TL::Type t : params)
        {
            llvm_params.push_back(get_llvm_type(t));
        }
        return llvm::FunctionType::get(get_llvm_type(t.returns()),
                                       llvm_params,
                                       /* isVarArg */ false);
    }
    else if (t.is_signed_integral() || t.is_bool())
    {
        return llvm::IntegerType::get(llvm_context, t.get_size() * 8);
    }
    else if (t.is_float())
    {
        return llvm_types.f32;
    }
    else if (t.is_double())
    {
        return llvm_types.f64;
    }
    else if (t.is_void())
    {
        return llvm_types.void_;
    }
    else if (t.is_array()
            && !t.array_requires_descriptor()
            && !t.array_is_vla())
    {
        // Statically sized arrays
        Nodecl::NodeclBase size = t.array_get_size();
        ERROR_CONDITION(!size.is_constant(), "Invalid size", 0);

        return llvm::ArrayType::get(get_llvm_type(t.array_element()),
                                    const_value_cast_to_8(size.get_constant()));
    }
    else if (t.is_array()
            && t.array_is_vla())
    {
        // Use the base element type
        return get_llvm_type(t.array_base_element());
    }
    else
    {
        internal_error("Cannot synthesize LLVM type for type '%s'",
                       print_declarator(t.get_internal_type()));
    }
}

llvm::Value* FortranLLVM::evaluate_size_of_array(TL::Type t)
{
    ERROR_CONDITION(!t.is_array(), "Invalid type", 0);
    if (t.array_requires_descriptor())
    {
        internal_error("Not yet implemented", 0);
    }
    else 
    {
        // VLA or constant-sized
        llvm::Value *val_size = eval_expression(t.array_get_size());
        t = t.array_element();

        while (t.is_array())
        {
            val_size = ir_builder->CreateMul(
                val_size, eval_expression(t.array_get_size()));
            t = t.array_element();
        }

        return val_size;
    }
}

void FortranLLVM::emit_variable(TL::Symbol sym)
{
    ERROR_CONDITION(!sym.is_variable(),
                    "Invalid symbol kind '%s'\n",
                    symbol_kind_name(sym.get_internal_symbol()));
    if (get_value(sym) != NULL)
        return;

    llvm::Value *array_size = nullptr;
    if (sym.get_type().is_array() && sym.get_type().array_is_vla())
    {
        // Emit size
        llvm::IRBuilderBase::InsertPoint previous_ip = change_to_allocating_block();

        bool block_was_terminated = get_current_block()->getTerminator() != nullptr;

        TL::Type t = sym.get_type();
        array_size = evaluate_size_of_array(t);

        if (get_current_block() != allocating_block()
                && block_was_terminated)
        {
            internal_error("We have split the allocating block. Not implemented yet", 0);
        }

        return_from_allocating_block(previous_ip);
    }

    llvm::Value *allocation = create_alloca(
        get_llvm_type(sym.get_type()), array_size, sym.get_name());
    map_symbol_to_value(sym, allocation);
}

void FortranLLVM::visit(const Nodecl::FortranPrintStatement& node)
{
    // TODO: Implement something fancier than list formating
    Nodecl::NodeclBase fmt = node.get_format();
    ERROR_CONDITION(!fmt.is<Nodecl::Text>()
                        || (fmt.as<Nodecl::Text>().get_text() != "*"),
                    "Only 'PRINT *' is implemented",
                    0);

    // Allocate data transfer structure
    llvm::Value *dt_parm
        = create_alloca(gfortran_rt.st_parameter_dt, nullptr, "dt_parm");

    // dt_parm.common.filename = "file";
    // dt_parm.common.line = "file";
    // dt_parm.common.flags = 128;
    ir_builder->CreateStore(
            ir_builder->CreateGlobalStringPtr(node.get_filename()),
        gep_for_field(
            gfortran_rt.st_parameter_dt, dt_parm, { "common", "filename" }));
    ir_builder->CreateStore(get_integer_value_32(node.get_line()),
                            gep_for_field(gfortran_rt.st_parameter_dt,
                                          dt_parm,
                                          { "common", "line" }));
    ir_builder->CreateStore(get_integer_value_32(128),
                            gep_for_field(gfortran_rt.st_parameter_dt,
                                          dt_parm,
                                          { "common", "flags" }));
    ir_builder->CreateStore(get_integer_value_32(6),
                            gep_for_field(gfortran_rt.st_parameter_dt,
                                          dt_parm,
                                          { "common", "unit" }));

    ir_builder->CreateCall(gfortran_rt.st_write, { dt_parm });

    Nodecl::List io_items = node.get_io_items().as<Nodecl::List>();
    // FIXME: Refactor
    for (Nodecl::NodeclBase n : io_items)
    {
        TL::Type t = n.get_type();
        if (fortran_is_character_type(t.get_internal_type()))
        {
            llvm::Value *expr = eval_expression(n);

            ir_builder->CreateCall(gfortran_rt.transfer_character_write,
                                   { dt_parm, expr, eval_sizeof(n) });
        }
        else if (t.no_ref().is_signed_integral())
        {
            llvm::Value *expr = eval_expression_to_memory(n);

            expr = ir_builder->CreatePointerCast(expr, llvm_types.ptr_i8);

            ir_builder->CreateCall(gfortran_rt.transfer_integer_write,
                                   { dt_parm, expr, eval_sizeof(n) });
        }
        else if (t.no_ref().is_float()
                || t.no_ref().is_double())
        {
            llvm::Value *expr = eval_expression_to_memory(n);

            expr = ir_builder->CreatePointerCast(expr, llvm_types.ptr_i8);

            ir_builder->CreateCall(gfortran_rt.transfer_real_write,
                                   { dt_parm, expr, eval_sizeof(n) });
        }
        else
        {
            internal_error("Type '%s' not yet implemented",
                           print_declarator(t.get_internal_type()));
        }
    }

    ir_builder->CreateCall(gfortran_rt.st_write_done, { dt_parm });
}

llvm::Value *FortranLLVM::gep_for_field(
    llvm::Type *struct_type,
    llvm::Value *addr,
    const std::vector<std::string> &access_fields)
{
    ERROR_CONDITION(access_fields.empty(), "Invalid empty set of fields", 0);

    llvm::Type *t = struct_type;
    std::vector<llvm::Value*> index_list;
    index_list.reserve(access_fields.size() + 1);
    index_list.push_back(get_integer_value_32(0));
    for (const auto &f : access_fields)
    {
        int idx = fields[t][f];

        index_list.push_back(get_integer_value_32(idx));

        t = llvm::cast<llvm::StructType>(t)->elements()[idx];
    }

    return ir_builder->CreateGEP(addr, index_list);
}

void FortranLLVM::initialize_llvm_types()
{
    llvm_types.i1 = llvm::Type::getInt1Ty(llvm_context);
    llvm_types.i8 = llvm::Type::getInt8Ty(llvm_context);
    llvm_types.i16 = llvm::Type::getInt16Ty(llvm_context);
    llvm_types.i32 = llvm::Type::getInt32Ty(llvm_context);
    llvm_types.i64 = llvm::Type::getInt64Ty(llvm_context);

    llvm_types.f32 = llvm::Type::getFloatTy(llvm_context);
    llvm_types.f64 = llvm::Type::getDoubleTy(llvm_context);

    llvm_types.void_ = llvm::Type::getVoidTy(llvm_context);

    llvm_types.ptr_i8 = llvm::Type::getInt8PtrTy(llvm_context);
    llvm_types.ptr_i32 = llvm::Type::getInt32PtrTy(llvm_context);
    llvm_types.ptr_i64 = llvm::Type::getInt64PtrTy(llvm_context);
}

// Based on ioparm.def from gfortran
#define IOPARM_LIST_FIELDS \
 IOPARM_START(common) \
  IOPARM (common,  flags,		0,	 int4) \
  IOPARM (common,  unit,		0,	 int4) \
  IOPARM (common,  filename,	0,	 pchar) \
  IOPARM (common,  line,		0,	 int4) \
  IOPARM (common,  iomsg,		1 << 6,  char2) \
  IOPARM (common,  iostat,	1 << 5,  pint4) \
 IOPARM_END(common) \
 IOPARM_START(dt) \
  IOPARM (dt,      common,	0,	 common) \
  IOPARM (dt,      rec,		1 << 9,  intio) \
  IOPARM (dt,      size,		1 << 10, pintio) \
  IOPARM (dt,      iolength,	1 << 11, pintio) \
  IOPARM (dt,      internal_unit_desc, 0,  parray) \
  IOPARM (dt,      format,	1 << 12, char1) \
  IOPARM (dt,      advance,	1 << 13, char2) \
  IOPARM (dt,      internal_unit,	1 << 14, char1) \
  IOPARM (dt,      namelist_name,	1 << 15, char2) \
  IOPARM (dt,      u,		0,	 pad) \
  IOPARM (dt,      id,		1 << 16, pint4) \
  IOPARM (dt,      pos,		1 << 17, intio) \
  IOPARM (dt,      asynchronous, 	1 << 18, char1) \
  IOPARM (dt,      blank,		1 << 19, char2) \
  IOPARM (dt,      decimal,	1 << 20, char1) \
  IOPARM (dt,      delim,		1 << 21, char2) \
  IOPARM (dt,      pad,		1 << 22, char1) \
  IOPARM (dt,      round,		1 << 23, char2) \
  IOPARM (dt,      sign,		1 << 24, char1) \
 IOPARM_END(dt)

void FortranLLVM::initialize_gfortran_runtime()
{
    typedef std::vector<llvm::Type *> TL;
    auto f_ioparm_type_int4 = [&, this](TL &t)
    {
        t.push_back(llvm_types.i32);
    };
    auto f_ioparm_type_intio = f_ioparm_type_int4;
    auto f_ioparm_type_pint4 = [&, this](TL &t)
    {
        t.push_back(llvm_types.ptr_i32);
    };
    auto f_ioparm_type_pchar = [&, this](TL &t)
    {
        t.push_back(llvm_types.ptr_i8);
    };
    auto f_ioparm_type_char1 = [&, this](TL &t)
    {
        t.push_back(llvm_types.i32);
        f_ioparm_type_pchar(t);
    };
    auto f_ioparm_type_char2 = [&, this](TL &t)
    {
        f_ioparm_type_pchar(t);
        t.push_back(llvm_types.i32);
    };
    auto f_ioparm_type_pintio = [&, this](TL &t)
    {
        t.push_back(llvm_types.i64);
    };
    auto f_ioparm_type_parray = f_ioparm_type_pchar;
    auto f_ioparm_type_pad = [&, this](TL &t)
    {
        const llvm::DataLayout &dl = current_module->getDataLayout();
        uint64_t size = 16 * dl.getPointerTypeSize(llvm_types.ptr_i8)
                        + 32 * dl.getTypeAllocSize(llvm_types.i32);
        t.push_back(llvm::ArrayType::get(llvm_types.i8, size));
    };


#define IOPARM_START(name) std::vector<llvm::Type *> name##_elements;
#define IOPARM_END(name)                                                 \
    llvm::Type *ioparm_type_##name = llvm::StructType::create(           \
        llvm_context, name##_elements, "st_parameter_" #name);           \
    __attribute__((unused)) auto f_ioparm_type_##name = [&, this](TL &t) \
    {                                                                    \
        t.push_back(ioparm_type_##name);                                 \
    };
#define IOPARM(name, _, __, type) f_ioparm_type_##type(name##_elements);
    IOPARM_LIST_FIELDS
#undef IOPARM
#undef IOPARM_END
#undef IOPARM_START

#define IOPARM_START(name)
#define IOPARM_END(name)
#define IOPARM(name, field_name, _, __) \
    fields[ioparm_type_##name].add_field(#field_name);
    IOPARM_LIST_FIELDS
#undef IOPARM
#undef IOPARM_END
#undef IOPARM_START

    this->gfortran_rt.st_parameter_common = ioparm_type_common;
    this->gfortran_rt.st_parameter_dt = ioparm_type_dt;

    this->gfortran_rt.st_write = llvm::Function::Create(
        llvm::FunctionType::get(llvm_types.void_,
                                { gfortran_rt.st_parameter_dt->getPointerTo() },
                                /* isVarArg */ false),
        llvm::GlobalValue::ExternalLinkage,
        "_gfortran_st_write",
        current_module.get());

    this->gfortran_rt.transfer_character_write = llvm::Function::Create(
        llvm::FunctionType::get(llvm_types.void_,
                                { gfortran_rt.st_parameter_dt->getPointerTo(),
                                  llvm_types.ptr_i8,
                                  llvm_types.i32 },
                                /* isVarArg */ false),
        llvm::GlobalValue::ExternalLinkage,
        "_gfortran_transfer_character_write",
        current_module.get());

    this->gfortran_rt.transfer_integer_write = llvm::Function::Create(
        llvm::FunctionType::get(llvm_types.void_,
                                { gfortran_rt.st_parameter_dt->getPointerTo(),
                                  llvm_types.ptr_i8,
                                  llvm_types.i32 },
                                /* isVarArg */ false),
        llvm::GlobalValue::ExternalLinkage,
        "_gfortran_transfer_integer_write",
        current_module.get());

    this->gfortran_rt.transfer_real_write = llvm::Function::Create(
        llvm::FunctionType::get(llvm_types.void_,
                                { gfortran_rt.st_parameter_dt->getPointerTo(),
                                  llvm_types.ptr_i8,
                                  llvm_types.i32 },
                                /* isVarArg */ false),
        llvm::GlobalValue::ExternalLinkage,
        "_gfortran_transfer_real_write",
        current_module.get());

    this->gfortran_rt.st_write_done = llvm::Function::Create(
        llvm::FunctionType::get(llvm_types.void_,
                                { gfortran_rt.st_parameter_dt->getPointerTo() },
                                /* isVarArg */ false),
        llvm::GlobalValue::ExternalLinkage,
        "_gfortran_st_write_done",
        current_module.get());

    this->gfortran_rt.set_args = llvm::Function::Create(
        llvm::FunctionType::get(
            llvm_types.void_,
            { llvm_types.i32,
              llvm_types.ptr_i8->getPointerTo() },
            /* isVarArg */ false),
        llvm::GlobalValue::ExternalLinkage,
        "_gfortran_set_args",
        current_module.get());

    this->gfortran_rt.set_options = llvm::Function::Create(
        llvm::FunctionType::get(llvm_types.void_,
                                { llvm_types.i32,
                                  llvm_types.ptr_i32 },
                                /* isVarArg */ false),
        llvm::GlobalValue::ExternalLinkage,
        "_gfortran_set_options",
        current_module.get());
}

void FortranLLVM::initialize_llvm_context()
{
    initialize_llvm_types();
    initialize_gfortran_runtime();
}

void FortranLLVM::emit_main(llvm::Function *fortran_program)
{
    // FIXME - This is hardcoded
    // FIXME - Move this to initialize_gfortran_runtime
    // static integer(kind=4) options.1[9] = {68, 1023, 0, 0, 1, 1, 0, 0, 31};
    llvm::ArrayType *options_type
        = llvm::ArrayType::get(llvm_types.i32, 9);
    std::vector<llvm::Constant *> options_values{
        get_integer_value_32(68), get_integer_value_32(1023), get_integer_value_32(0),
        get_integer_value_32(0),  get_integer_value_32(1),    get_integer_value_32(1),
        get_integer_value_32(0),  get_integer_value_32(0),    get_integer_value_32(31)
    };

    llvm::GlobalVariable *options = new llvm::GlobalVariable(
        *current_module,
        options_type,
        /* isConstant */ true,
        llvm::GlobalValue::PrivateLinkage,
        llvm::ConstantArray::get(options_type, options_values));

    llvm::Constant *c = current_module->getOrInsertFunction(
        "main",
        llvm::FunctionType::get(
            llvm_types.i32,
            { llvm_types.i32,
              llvm_types.ptr_i8->getPointerTo() },
            /* isVarArg */ false),
        /* no attributes so far */ llvm::AttributeSet());

    llvm::Function *fun = llvm::cast<llvm::Function>(c);

    set_current_function(fun);
    llvm::BasicBlock *entry_basic_block = llvm::BasicBlock::Create(llvm_context, "entry", fun);
    set_current_block(entry_basic_block);
    push_allocating_block(entry_basic_block);

    std::vector<llvm::Value *> main_args;
    for (llvm::Argument &v : fun->args())
    {
        main_args.push_back(&v);
    }
    ir_builder->CreateCall(gfortran_rt.set_args, main_args);

    ir_builder->CreateCall(
        gfortran_rt.set_options,
        { get_integer_value_32(9),
          ir_builder->CreatePointerCast(options, llvm_types.ptr_i32) });

    ir_builder->CreateCall(fortran_program, {});

    ir_builder->CreateRet(get_integer_value_32(0));

    pop_allocating_block();
    clear_current_function();
}

llvm::Constant *FortranLLVM::get_integer_value_N(int64_t v, llvm::Type* t, int bits)
{
    return llvm::Constant::getIntegerValue(t, llvm::APInt(bits, v, /* signed */ 1));
}

llvm::Constant *FortranLLVM::get_integer_value(int64_t v, TL::Type t)
{
    ERROR_CONDITION(!t.is_signed_integral() && !t.is_bool(),
                    "Must be a signed integral or boolean",
                    0);
    return get_integer_value_N(v, get_llvm_type(t), t.get_size() * 8);
}

llvm::Constant *FortranLLVM::get_integer_value_32(int64_t v)
{
    return get_integer_value_N(v, llvm_types.i32, 32);
}

llvm::Constant *FortranLLVM::get_integer_value_64(int64_t v)
{
    return get_integer_value_N(int64_t(v), llvm_types.i64, 64);
}

}


EXPORT_PHASE(Codegen::FortranLLVM)
