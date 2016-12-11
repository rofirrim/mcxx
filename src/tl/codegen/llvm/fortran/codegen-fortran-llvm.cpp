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

    // FIXME: Finish function. Use proper value!
    ir_builder->CreateRet(nullptr);

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

void FortranLLVM::visit(const Nodecl::IfElseStatement& node)
{
    Nodecl::NodeclBase condition = node.get_condition();
    Nodecl::NodeclBase then_node = node.get_then();
    Nodecl::NodeclBase else_node = node.get_else();

    llvm::Value *cond_val = eval_expression(condition);

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
        vstep = getIntegerValue(1, ind_var.get_symbol().get_type());
    else
        vstep = eval_expression(step);

    ir_builder->CreateStore(vlower, vind_var);

    llvm::BasicBlock *block_check = llvm::BasicBlock::Create(llvm_context, "loop.check", get_current_function());
    llvm::BasicBlock *block_body = llvm::BasicBlock::Create(llvm_context, "loop.body", get_current_function());
    llvm::BasicBlock *block_end = llvm::BasicBlock::Create(llvm_context, "loop.end", get_current_function());

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

    void visit(const Nodecl::Conversion& node)
    {
        // A conversion coalesces more than one conversion including a "load"
        // For now just check that this is a pure load
        llvm::Value *vnest = llvm_visitor->eval_expression(node.get_nest());
        bool is_load = 
            node.get_type().is_same_type(node.get_nest().get_type().no_ref());
        ERROR_CONDITION(!is_load, "Not yet implemented", 0);

        value = llvm_visitor->ir_builder->CreateLoad(vnest);
    }

    void visit(const Nodecl::Assignment& node)
    {
        llvm::Value *vlhs = llvm_visitor->eval_expression(node.get_lhs());
        llvm::Value *vrhs = llvm_visitor->eval_expression(node.get_rhs());

        value = llvm_visitor->ir_builder->CreateStore(vrhs, vlhs);
    }

    // void visit(const Nodecl::ObjectInit& node);
    // void visit(const Nodecl::Plus& node);
    // void visit(const Nodecl::Neg& node);
    // void visit(const Nodecl::LogicalNot& node);
    // void visit(const Nodecl::Mul& node);
    // void visit(const Nodecl::Div& node);
    // void visit(const Nodecl::Mod& node);
    void visit(const Nodecl::Add& node)
    {
        if (node.get_type().is_signed_integral())
        {
            llvm::Value *vlhs = llvm_visitor->eval_expression(node.get_lhs());
            llvm::Value *vrhs = llvm_visitor->eval_expression(node.get_rhs());

            value = llvm_visitor->ir_builder->CreateAdd(vlhs, vrhs);
        }
        else
        {
            internal_error(
                "Not implemented yet '%s'\n",
                print_declarator(node.get_type().get_internal_type()));
        }
    }
    // void visit(const Nodecl::Minus& node);
    // void visit(const Nodecl::LowerThan& node);
    // void visit(const Nodecl::LowerOrEqualThan& node);
    void visit(const Nodecl::GreaterThan& node)
    {
        if (node.get_lhs().get_type().is_signed_integral()
                && node.get_rhs().get_type().is_signed_integral())
        {
            llvm::Value *vlhs = llvm_visitor->eval_expression(node.get_lhs());
            llvm::Value *vrhs = llvm_visitor->eval_expression(node.get_rhs());

            value = llvm_visitor->ir_builder->CreateICmpSGT(
                    vlhs,
                    vrhs);
        }
        else
        {
            internal_error(
                "Not implemented yet '%s' <?> '%s\n",
                print_declarator(node.get_lhs().get_type().get_internal_type()),
                print_declarator(node.get_rhs().get_type().get_internal_type()));
        }
    }
    // void visit(const Nodecl::GreaterOrEqualThan& node);
    // void visit(const Nodecl::LogicalAnd& node);
    // void visit(const Nodecl::LogicalOr& node);
    // void visit(const Nodecl::Power& node);
    // void visit(const Nodecl::Concat& node);
    // void visit(const Nodecl::ClassMemberAccess& node);
    // void visit(const Nodecl::Range& node);
    void visit(const Nodecl::StringLiteral& node)
    {
        std::string str = node.get_text();
        str = str.substr(1, str.size() - 2);
        value = llvm_visitor->ir_builder->CreateGlobalStringPtr(str);
    }
    // void visit(const Nodecl::Text& node);
    // void visit(const Nodecl::StructuredValue& node);
    // void visit(const Nodecl::BooleanLiteral& node);
    void visit(const Nodecl::IntegerLiteral &node)
    {
        value = llvm_visitor->getIntegerValue(
            // FIXME: INTEGER(16)
            const_value_cast_to_8(node.get_constant()),
            node.get_type());
    }

    // void visit(const Nodecl::ComplexLiteral& node);
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
    // void visit(const Nodecl::Symbol& node);
    // void visit(const Nodecl::Equal& node);
    // void visit(const Nodecl::Different& node);
    // void visit(const Nodecl::Dereference& node);
    // void visit(const Nodecl::Reference& node);
    // void visit(const Nodecl::ParenthesizedExpression& node);
    void visit(const Nodecl::ArraySubscript& node)
    {
        Nodecl::NodeclBase subscripted = node.get_subscripted();
        TL::Type subscripted_type = subscripted.get_type();
        TL::Type subscripted_type_noref = subscripted_type.no_ref();

        ERROR_CONDITION(!subscripted_type_noref.is_array(),
                "Expecting an array here", 0);

        if (subscripted_type_noref.array_requires_descriptor()
                || subscripted_type_noref.array_is_vla())
            internal_error("Not implemented yet", 0);

        Nodecl::List subscripts = node.get_subscripts().as<Nodecl::List>();
        std::vector<llvm::Value*> index_list;
        index_list.reserve(subscripts.size() + 1);
        index_list.push_back(llvm_visitor->getIntegerValue32(0));

        TL::Type current_array_type = subscripted_type_noref;
        for (Nodecl::NodeclBase index : subscripts)
        {
            llvm::Value *idx = llvm_visitor->ir_builder->CreateSExtOrTrunc(
                llvm_visitor->eval_expression(index),
                llvm_visitor->llvm_types.i64);

            Nodecl::NodeclBase lower, upper;
            current_array_type.array_get_bounds(lower, upper);
            llvm::Value *base = llvm_visitor->ir_builder->CreateSExtOrTrunc(
                llvm_visitor->eval_expression(lower),
                llvm_visitor->llvm_types.i64);

            idx = llvm_visitor->ir_builder->CreateSub(idx, base);
            index_list.push_back(idx);

            current_array_type = current_array_type.array_element();
        }

        llvm::Value* subscripted_val = llvm_visitor->eval_expression(subscripted);
        value = llvm_visitor->ir_builder->CreateGEP(subscripted_val, index_list);
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

            llvm::Value *varg;
            if (it_param->is_any_reference())
            {
                varg = llvm_visitor->eval_expression_to_memory(arg);
            }
            else
            {
                varg = llvm_visitor->eval_expression(arg);
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

llvm::Value *FortranLLVM::create_alloca(llvm::Type *t,
                                   llvm::Value *array_size,
                                   const llvm::Twine &name)
{
    llvm::IRBuilderBase::InsertPoint savedIP = ir_builder->saveIP();

    llvm::BasicBlock *alloca_bb = allocating_block();

    // FIXME: I'm pretty sure it is possible to do this better.
    llvm::BasicBlock::iterator bb_it = alloca_bb->begin();
    while (bb_it != alloca_bb->end()
            && llvm::isa<llvm::AllocaInst>(*bb_it))
        bb_it++;

    ir_builder->SetInsertPoint(alloca_bb, bb_it);
    llvm::Value *tmp
        = ir_builder->CreateAlloca(t, array_size, name);

    ir_builder->restoreIP(savedIP);
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

// class FortranVisitorLLVMExpressionSizeof : public FortranVisitorLLVMExpressionBase
// {
// };

// FIXME - Maybe this should return i64?
llvm::Value *FortranLLVM::eval_sizeof(Nodecl::NodeclBase n)
{
    return getIntegerValue32(n.get_type().no_ref().get_size());
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
    else
    {
        internal_error("Cannot convert type '%s' to LLVM type",
                       print_declarator(t.get_internal_type()));
    }
}

void FortranLLVM::emit_variable(TL::Symbol sym)
{
    ERROR_CONDITION(!sym.is_variable(),
                    "Invalid symbol kind '%s'\n",
                    symbol_kind_name(sym.get_internal_symbol()));
    if (get_value(sym) != NULL)
        return;

    llvm::Value *allocation = create_alloca(get_llvm_type(sym.get_type()),
                                           /* array_size */ nullptr,
                                           sym.get_name());
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
    ir_builder->CreateStore(getIntegerValue32(node.get_line()),
                            gep_for_field(gfortran_rt.st_parameter_dt,
                                          dt_parm,
                                          { "common", "line" }));
    ir_builder->CreateStore(getIntegerValue32(128),
                            gep_for_field(gfortran_rt.st_parameter_dt,
                                          dt_parm,
                                          { "common", "flags" }));
    ir_builder->CreateStore(getIntegerValue32(6),
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
    index_list.push_back(getIntegerValue32(0));
    for (const auto &f : access_fields)
    {
        int idx = fields[t][f];

        index_list.push_back(getIntegerValue32(idx));

        t = llvm::cast<llvm::StructType>(t)->elements()[idx];
    }

    return ir_builder->CreateGEP(addr, index_list);
}

void FortranLLVM::initialize_llvm_types()
{
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
        getIntegerValue32(68), getIntegerValue32(1023), getIntegerValue32(0),
        getIntegerValue32(0),  getIntegerValue32(1),    getIntegerValue32(1),
        getIntegerValue32(0),  getIntegerValue32(0),    getIntegerValue32(31)
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
        { getIntegerValue32(9),
          ir_builder->CreatePointerCast(options, llvm_types.ptr_i32) });

    ir_builder->CreateCall(fortran_program, {});

    ir_builder->CreateRet(getIntegerValue32(0));

    pop_allocating_block();
    clear_current_function();
}

llvm::Constant *FortranLLVM::getIntegerValueN(int64_t v, llvm::Type* t, int bits)
{
    return llvm::Constant::getIntegerValue(t, llvm::APInt(bits, v, /* signed */ 1));
}

llvm::Constant *FortranLLVM::getIntegerValue(int64_t v, TL::Type t)
{
    ERROR_CONDITION(!t.is_signed_integral(), "Must be a signed integral", 0);
    return getIntegerValueN(v, get_llvm_type(t), t.get_size() * 8);
}

llvm::Constant *FortranLLVM::getIntegerValue32(int64_t v)
{
    return getIntegerValueN(v, llvm_types.i32, 32);
}

llvm::Constant *FortranLLVM::getIntegerValue64(int64_t v)
{
    return getIntegerValueN(int64_t(v), llvm_types.i64, 64);
}

}


EXPORT_PHASE(Codegen::FortranLLVM)
