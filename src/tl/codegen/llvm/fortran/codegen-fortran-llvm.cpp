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

#include "config.h"
#include "filename.h"
#include "cxx-driver-build-info.h"
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

    current_module->addModuleFlag(llvm::Module::Error, "PIC Level", 2);
    current_module->addModuleFlag(llvm::Module::Warning, "Debug Info Version",
                              llvm::DEBUG_METADATA_VERSION);
    current_module->addModuleFlag(llvm::Module::Warning, "Dwarf Version", 4);

    ERROR_CONDITION(CURRENT_CONFIGURATION->type_environment->triplet == NULL, "Invalid triple", 0);
    std::string default_triple = CURRENT_CONFIGURATION->type_environment->triplet;
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
    md_builder
        = std::unique_ptr<llvm::MDBuilder>(new llvm::MDBuilder(llvm_context));
    dbg_builder = std::unique_ptr<llvm::DIBuilder>(
        new llvm::DIBuilder(*current_module));

    std::string base = give_basename(node.get_filename().c_str());
    std::string dir = give_dirname(node.get_filename().c_str());
    dbg_info.reset();
    dbg_info.file = dbg_builder->createFile(base, dir);
    /* llvm::DICompileUnit *dbg_compile_unit = */ dbg_builder->createCompileUnit(llvm::dwarf::DW_LANG_Fortran95,
        dbg_info.file,
        PACKAGE " " VERSION " (" MCXX_BUILD_VERSION ")",
        /* isOptimized */ false,
        /* Flags */ "",
        /* RuntimeVersion */ 0);

    initialize_llvm_context();

    push_debug_scope(dbg_info.file);
    walk(node.get_top_level());
    pop_debug_scope();
    ERROR_CONDITION(!dbg_info.stack_debug_scope.empty(), "Stack of debug scopes is not empty", 0);

    dbg_builder->finalize();

    llvm::raw_os_ostream ros(*file);
    current_module->print(ros,
                          /* AssemblyAnnotationWriter */ nullptr);

    std::swap(old_module, current_module);
}

// This strategy is not very efficient when there are many nested scopes as
// several subtrees will be traversed many times but it makes the
// implementation much cleaner and Fortran rarely has more than one lexical
// scope per function.
class FortranVisitorLLVMEmitVariables : public Nodecl::ExhaustiveVisitor<void>
{
    FortranLLVM *llvm_visitor;
    TL::Scope current_scope;
    TL::ObjectList<TL::Symbol> to_emit;

    void check_symbol(TL::Symbol sym)
    {
        if (!sym.is_from_module()
            && sym.is_variable()
            && sym.get_scope() == current_scope)
        {
            // Check boundaries of arrays first in case
            // they require a symbol not yet seen
            TL::Type t = sym.get_type().no_ref();
            while (t.is_fortran_array())
            {
                Nodecl::NodeclBase lower, upper;
                t.array_get_bounds(lower, upper);
                walk(lower);
                walk(upper);
                t = t.array_element();
            }

            // TODO: CHARACTER(LEN=*)
            // if (t.is_fortran_character())
            // {
            // }

            // Saved expressions are symbols that will likely
            // require other symbols
            if (sym.is_saved_expression())
                walk(sym.get_value());

            to_emit.insert(sym);
        }
    }
    public:
    FortranVisitorLLVMEmitVariables(FortranLLVM *llvm_visitor, TL::Scope current_scope)
        : llvm_visitor(llvm_visitor), current_scope(current_scope)
    {
    }

    void visit(const Nodecl::Symbol &node)
    {
        TL::Symbol sym = node.get_symbol();
        check_symbol(sym);
    }

    void visit(const Nodecl::ObjectInit &node)
    {
        TL::Symbol sym = node.get_symbol();
        check_symbol(sym);
    }

    void emit_variables(const Nodecl::NodeclBase &node)
    {
        to_emit.clear();
        walk(node);
        for (TL::Symbol sym : to_emit)
        {
            llvm_visitor->emit_variable(sym);
        }
    }
};


void FortranLLVM::visit(const Nodecl::FunctionCode &node)
{
    // Create a function with the proper type
    TL::Symbol sym = node.get_symbol();

    TL::Type function_type;
    std::string mangled_name;

    llvm::AttributeSet attributes;

    if (sym.is_fortran_main_program())
    {
        mangled_name = "MAIN__";
        function_type = TL::Type::get_void_type().get_function_returning(
                TL::ObjectList<TL::Type>());
        attributes = attributes.addAttribute(llvm_context,
                llvm::AttributeSet::FunctionIndex,
                llvm::Attribute::NoRecurse);
    }
    else
    {
        mangled_name = fortran_mangle_symbol(sym.get_internal_symbol());
        function_type = sym.get_type();
    }

    if (sym.is_module_procedure())
    {
        llvm::DIModule *module = get_module(sym.in_module().get_name());
        push_debug_scope(module);
    }

    attributes = attributes.addAttribute(llvm_context,
            llvm::AttributeSet::FunctionIndex,
            llvm::Attribute::UWTable);
    attributes = attributes.addAttribute(llvm_context,
            llvm::AttributeSet::FunctionIndex,
            llvm::Attribute::NoUnwind);
    // attributes = attributes.addAttribute(llvm_context,
    //         llvm::AttributeSet::FunctionIndex,
    //         "no-frame-pointer-elim", "true");
    // attributes = attributes.addAttribute(llvm_context,
    //         llvm::AttributeSet::FunctionIndex,
    //         "no-frame-pointer-elim-non-leaf");

    llvm::Type *llvm_function_type = get_llvm_type(function_type);

    llvm::Constant *c = current_module->getOrInsertFunction(
        mangled_name,
        llvm::cast<llvm::FunctionType>(llvm_function_type),
        attributes);

    llvm::Function *fun = llvm::cast<llvm::Function>(c);

    llvm::DINode::DIFlags flags = llvm::DINode::FlagPrototyped;
    if (sym.is_fortran_main_program())
        flags |= llvm::DINode::FlagMainSubprogram;
    llvm::DISubroutineType* dbg_type = llvm::cast<llvm::DISubroutineType>(get_debug_info_type(function_type));
    llvm::DISubprogram* dbg_subprogram = dbg_builder->createFunction(
        get_debug_scope(),
        sym.get_name(),
        mangled_name,
        dbg_info.file,
        sym.get_line(),
        dbg_type,
        /* isLocalToUnit */ false,
        /* isDefinition */ true,
        /* ScopeLine */ sym.get_line(),
        flags);
    fun->setSubprogram(dbg_subprogram);
    dbg_info.function = dbg_subprogram;

    push_debug_scope(dbg_subprogram);

    // Set argument names
    TL::ObjectList<TL::Symbol> related_symbols = sym.get_related_symbols();
    TL::ObjectList<TL::Symbol>::iterator related_symbols_it
        = related_symbols.begin();

    llvm::Function::ArgumentListType &llvm_fun_args = fun->getArgumentList();
    llvm::Function::ArgumentListType::iterator llvm_fun_args_it
        = llvm_fun_args.begin();

    // We need to handle this context here due to debug info of parameters.
    // A lexical scope should not be created for top level variables.
    Nodecl::Context context = node.get_statements().as<Nodecl::Context>();

    // Do not place this earlier because we need to make sure that a
    // DILocalScope is in the scope stack before we use this.
    FortranLLVM::TrackLocation loc(this, node);

    // Create entry block
    set_current_function(fun);
    llvm::BasicBlock *entry_basic_block
        = llvm::BasicBlock::Create(llvm_context, "entry", fun);
    set_current_block(entry_basic_block);

    // Register parameters
    clear_mappings();
    int dbg_argno = 1;
    while (related_symbols_it != related_symbols.end()
           && llvm_fun_args_it != llvm_fun_args.end())
    {
        llvm::Value *v = &*llvm_fun_args_it;
        TL::Symbol s = *related_symbols_it;

        if (!s.get_type().no_ref().is_fortran_array()
                && !s.get_type().is_any_reference())
        {
            // VALUE dummy argument
            // Emit a temporary storage for it
            v->setName(s.get_name() + ".value");
            v = make_temporary(v);
        }
        else if (!s.is_optional())
        {
            // We allow OPTIONAL be NULL (I think implementing OPTIONAL this way violates the standard)
            llvm::AttributeSet attrs = fun->getAttributes();
            attrs = attrs.addAttribute(llvm_context, dbg_argno, llvm::Attribute::NonNull);
            if (!s.get_type().no_ref().is_fortran_array())
                attrs = attrs.addAttribute(llvm_context, {(unsigned)dbg_argno},
                        llvm::Attribute::get(llvm_context,
                            llvm::Attribute::Dereferenceable, 
                            s.get_type().no_ref().get_size()));
            fun->setAttributes(attrs);
        }

        v->setName(s.get_name());
        map_symbol_to_value(s, v);

        std::vector<int64_t> dbg_expr_ops;
        llvm::DIExpression *dbg_expr = dbg_builder->createExpression(dbg_expr_ops);

        llvm::DILocalVariable *dbg_param =
            dbg_builder->createParameterVariable(get_debug_scope(),
                    s.get_name(),
                    dbg_argno,
                    dbg_info.file,
                    s.get_line(),
                    get_debug_info_type(s.get_type()));
        dbg_builder->insertDeclare(v,
                dbg_param,
                dbg_expr,
                llvm::DILocation::get(llvm_context,
                    s.get_line(),
                    s.get_column(),
                    get_debug_scope()),
                ir_builder->GetInsertBlock());
                
        dbg_argno++;

        related_symbols_it++;
        llvm_fun_args_it++;
    }
    ERROR_CONDITION(related_symbols_it != related_symbols.end()
                        || llvm_fun_args_it != llvm_fun_args.end(),
                    "Mismatch between TL and llvm::Arguments",
                    0);

    // Emit top level variables
    FortranVisitorLLVMEmitVariables e(this,
        nodecl_get_decl_context(context.get_internal_nodecl()));
    e.emit_variables(context.get_in_context());
    
    // Emit statements
    walk(context.get_in_context());

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
        // Make sure it has been emitted in case it has not been referenced at all
        emit_variable(return_variable);
        llvm::Value* addr_ret_val = get_value(return_variable);
        llvm::Value* value_ret_val = ir_builder->CreateLoad(addr_ret_val);
        ir_builder->CreateRet(value_ret_val);
    }

    pop_debug_scope(); // subroutine
    dbg_info.function = nullptr;

    if (sym.is_module_procedure())
    {
        pop_debug_scope(); // module
    }

    clear_current_function();

    if (sym.is_fortran_main_program())
    {
        emit_main(fun);
    }
}

void FortranLLVM::visit(const Nodecl::Context &node)
{
    FortranLLVM::TrackLocation loc(this, node);

    // This node does not usually appear in Fortran except in the top level of
    // a FunctionCode or if using BLOCK .. END BLOCK. For the FunctionCode case, we handle it
    // in the visitor of FunctionCode so this code will never be run by that case.
    llvm::DILexicalBlock *lexical_block = dbg_builder->createLexicalBlock(get_debug_scope(), dbg_info.file,
            node.get_line(), node.get_column());
    push_debug_scope(lexical_block);

    // Emit variables (if any)
    FortranVisitorLLVMEmitVariables e(this,
        nodecl_get_decl_context(node.get_internal_nodecl()));
    e.emit_variables(node.get_in_context());

    walk(node.get_in_context());
    pop_debug_scope();
}

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
        FortranLLVM::TrackLocation loc(llvm_visitor, node);
        value = llvm_visitor->get_value(node.get_symbol());
    }

    llvm::Value *scalar_conversion(TL::Type dest, TL::Type orig, llvm::Value* value_nest)
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

    void visit(const Nodecl::Conversion& node)
    {
        FortranLLVM::TrackLocation loc(llvm_visitor, node);

        TL::Type dest = node.get_type();
        TL::Type orig = node.get_nest().get_type();

        llvm::Value *nest_value = llvm_visitor->eval_expression(node.get_nest());

        if (dest.is_fortran_array()
                && orig.no_ref().is_fortran_array())
        {
            // We do not represent values of array, so we let the address pass-through
            // FIXME: Arrays with region that are demoted to non-region arrays
            ERROR_CONDITION(
                !dest.array_element().is_same_type(orig.no_ref().array_element()),
                "array-wise value conversions not implemented yet",
                0);
            value = nest_value;
        }
        else if (dest.is_fortran_character()
            && orig.is_any_reference()
            && orig.no_ref().is_fortran_character())
        {
            // We do not represent values of array, so we let the address pass-through
            value = nest_value;
        }
        else if (dest.is_fortran_array()
                && !orig.no_ref().is_fortran_array())
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
        return n.get_type().is_fortran_array()
               && !n.no_conv().get_type().no_ref().is_fortran_array();
    }

    struct LoopInfoOp
    {
        std::vector<llvm::Value *> idx_var;
        std::vector<llvm::BasicBlock *> block_check;
        std::vector<llvm::BasicBlock *> block_body;
        std::vector<llvm::BasicBlock *> block_end;
    };

    void create_loop_header_for_array_op(
            Nodecl::NodeclBase expr,
            TL::Type t,
            llvm::Value *addr,
            /* out */
            LoopInfoOp &loop_info_op
            )
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

            llvm::Value *idx_var = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
            loop_info_op.idx_var.push_back(idx_var);

            llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0), idx_var);

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
                        llvm_visitor->array_descriptor_addr_dim_stride(addr,
                                                                       i));
                vupper = llvm_visitor->ir_builder->CreateLoad(
                    llvm_visitor->array_descriptor_addr_dim_upper_bound(addr,
                                                                        i));
                vlower = llvm_visitor->ir_builder->CreateLoad(
                    llvm_visitor->array_descriptor_addr_dim_lower_bound(addr,
                                                                        i));
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
                    llvm_visitor->ir_builder->CreateSub(vupper, vlower),
                    vstride),
                vstride);

            llvm_visitor->ir_builder->CreateBr(block_check);
            llvm_visitor->set_current_block(block_check);

            llvm::Value *vcheck = llvm_visitor->ir_builder->CreateICmpSLT(
                llvm_visitor->ir_builder->CreateLoad(idx_var), loop_upper);
            llvm_visitor->ir_builder->CreateCondBr(
                vcheck, block_body, block_end);

            llvm_visitor->set_current_block(block_body);

            t = t.array_element();
        }
    }

    void create_loop_footer_for_array_op(TL::Type t,
            const LoopInfoOp &loop_info_op
            )
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
    llvm::Value *address_array_ith_element_via_descriptor(
        const Nodecl::NodeclBase &array,
        TL::Type t,
        llvm::Value *descr_address,
        const std::vector<llvm::Value *> indexes)
    {
        bool unit_stride = llvm_visitor->array_expression_will_use_unit_stride(array);
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
                llvm::Value* val_stride = llvm_visitor->ir_builder->CreateLoad(
                    llvm_visitor->array_descriptor_addr_dim_stride(
                        descr_address, i));
                offset = llvm_visitor->ir_builder->CreateMul(offset, val_stride);
            }
            offset_list.push_back(offset);

            llvm::Value *val_lower = llvm_visitor->ir_builder->CreateLoad(
                llvm_visitor->array_descriptor_addr_dim_lower_bound(
                    descr_address, i));
            llvm::Value *val_upper = llvm_visitor->ir_builder->CreateLoad(
                llvm_visitor->array_descriptor_addr_dim_upper_bound(
                    descr_address, i));

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

        ERROR_CONDITION(t.is_fortran_array(),
                        "Should not be an array here",
                        0);

        llvm::Value *base_address = llvm_visitor->ir_builder->CreateLoad(
            llvm_visitor->array_descriptor_addr_base_addr(descr_address));

        base_address = llvm_visitor->ir_builder->CreatePointerCast(
            base_address,
            llvm::PointerType::get(
                llvm_visitor->get_llvm_type(t.fortran_array_base_element()),
                /* AddressSpace */ 0));

        return llvm_visitor->ir_builder->CreateGEP(base_address, { val_addr });
    }

    llvm::Value *address_array_ith_element_via_pointer_arithmetic(
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
                        indexes[i],
                        llvm_visitor->eval_expression(region_stride)));

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
        ERROR_CONDITION(t.is_fortran_array(),
                        "Should not be an array here",
                        0);

        return llvm_visitor->ir_builder->CreateGEP(
                base_address, { val_addr } );
    }

    llvm::Value *address_array_ith_element(
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
        ERROR_CONDITION(rank != (int)indexes.size(),
                        "Mismatch between indexes and rank!\n",
                        0);

        if (t.array_requires_descriptor())
            return address_array_ith_element_via_descriptor(array, t, base_address, indexes);
        else
            return address_array_ith_element_via_pointer_arithmetic(t, base_address, indexes);
    }

    std::vector<llvm::Value* > derref_indexes(const std::vector<llvm::Value* > v)
    {
        std::vector<llvm::Value*> ret(v.begin(), v.end());
        for (auto &v : ret)
        {
            v = llvm_visitor->ir_builder->CreateLoad(v);
        }
        return ret;
    }


    template <typename Creator>
    void array_assignment(const Nodecl::NodeclBase &lhs,
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
        std::vector<llvm::Value*> idx_val = derref_indexes(loop_info_op.idx_var);
        llvm::Value *lhs_addr_element = address_array_ith_element(lhs, lhs_type, lhs_addr, idx_val);
        llvm::Value *rhs_addr_element = address_array_ith_element(rhs, rhs_type, rhs_addr, idx_val);
        create_store(llvm_visitor->ir_builder->CreateLoad(rhs_addr_element),
                     lhs_addr_element);
        create_loop_footer_for_array_op(rhs_type, loop_info_op);
    }

    typedef std::function<void (llvm::Value *addr, llvm::Value *value)> AssigOp;
    AssigOp get_assig_op(TL::Type lhs_type, TL::Type rhs_type)
    {
        if (lhs_type.no_ref().is_fortran_character()
            && rhs_type.no_ref().is_fortran_character())
        {
            return [&, lhs_type, rhs_type, this](llvm::Value * rhs,
                                                      llvm::Value * lhs)
            {
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

    void visit(const Nodecl::Assignment& node)
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
    void visit(const Nodecl::BooleanLiteral& node)
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

    void visit(const Nodecl::ComplexLiteral& node)
    {
        FortranLLVM::TrackLocation loc(llvm_visitor, node);

        TL::Type base_type = node.get_type().complex_get_base_type();
        const_value_t *cval = node.get_constant();

        llvm::Value *real_value, *imag_value;
        if (base_type.is_float())
        {
            real_value = value = llvm::ConstantFP::get(
                llvm_visitor->llvm_context,
                llvm::APFloat(const_value_cast_to_float(const_value_complex_get_real_part(cval))));
            imag_value = value = llvm::ConstantFP::get(
                llvm_visitor->llvm_context,
                llvm::APFloat(const_value_cast_to_float(const_value_complex_get_imag_part(cval))));
        }
        else if (base_type.is_double())
        {
            real_value = value = llvm::ConstantFP::get(
                llvm_visitor->llvm_context,
                llvm::APFloat(const_value_cast_to_double(const_value_complex_get_real_part(cval))));
            imag_value = value = llvm::ConstantFP::get(
                llvm_visitor->llvm_context,
                llvm::APFloat(const_value_cast_to_double(const_value_complex_get_imag_part(cval))));
        }
        else
        {
            internal_error("Unexpected type '%s'",
                           print_declarator(base_type.get_internal_type()));
        }

        value = llvm::ConstantVector::get({ llvm::cast<llvm::Constant>(real_value), llvm::cast<llvm::Constant>(imag_value) });
    }

    void visit(const Nodecl::ObjectInit& node)
    {
        std::cerr << "Unhandled object init. Symbol = " << node.get_symbol().get_name() << " " << symbol_kind_name(node.get_symbol().get_internal_symbol()) << std::endl;
    }

    void visit(const Nodecl::Neg &node)
    {
        FortranLLVM::TrackLocation loc(llvm_visitor, node);

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
        FortranLLVM::TrackLocation loc(llvm_visitor, node);

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
        FortranLLVM::TrackLocation loc(llvm_visitor, node);

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
        FortranLLVM::TrackLocation loc(llvm_visitor, node);

        Nodecl::NodeclBase rhs = node.get_rhs();
        TL::Type rhs_type = rhs.get_type();

        value = llvm_visitor->eval_expression(rhs);

        // select x, 0, 1
        value = llvm_visitor->ir_builder->CreateSelect(
            llvm_visitor->ir_builder->CreateZExtOrTrunc(
                value, llvm_visitor->llvm_types.i1),
            llvm_visitor->get_integer_value(0, rhs_type),
            llvm_visitor->get_integer_value(1, rhs_type));
    }

    template <typename Create>
    llvm::Value *arithmetic_binary_op_elemental_intrinsic(
        Nodecl::NodeclBase lhs,
        Nodecl::NodeclBase rhs,
        llvm::Value *lhs_val,
        llvm::Value *rhs_val,
        Create create)
    {
        return create(lhs, rhs, lhs_val, rhs_val);
    }

    llvm::Value *compute_offset_from_linear_element(TL::Type t, llvm::Value* idx_value)
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
                        offset = llvm_visitor->ir_builder->CreateAdd(
                            vregion_lower, idx_value);
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
                    llvm::Value *vstride
                        = llvm_visitor->eval_expression(stride);
                    offset = llvm_visitor->ir_builder->CreateAdd(
                        vregion_upper,
                        llvm_visitor->ir_builder->CreateMul(idx_value,
                                                            vstride));
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
                offset
                    = llvm_visitor->ir_builder->CreateSub(offset, varray_lower);

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
    void arithmetic_binary_operator_array(const Nodecl::NodeclBase &node,
                                          const Nodecl::NodeclBase &lhs,
                                          const Nodecl::NodeclBase &rhs,
                                          Create create)
    {
        TL::Type node_type = node.get_type();

        ERROR_CONDITION(
            !node_type.is_fortran_array(), "The result must be an array!\n", 0);

        TL::Type node_element_type = node_type.array_base_element();

        llvm::Value *array_size
            = llvm_visitor->eval_elements_of_array(node, node_type, /* addr */ nullptr);

        // Allocate space for the result
        llvm::Value *result_addr = llvm_visitor->ir_builder->CreateAlloca(
                llvm_visitor->get_llvm_type(node_element_type), array_size);

        // Index inside the contiguous result array. This is used for looping.
        llvm::Value *result_idx_addr = llvm_visitor->ir_builder->CreateAlloca(
            llvm_visitor->llvm_types.i64);
        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->get_integer_value_64(0), result_idx_addr);

        LoopInfoOp loop_info_op;
        create_loop_header_for_array_op(node, node_type, /* addr */ nullptr, loop_info_op);
        std::vector<llvm::Value*> idx_val = derref_indexes(loop_info_op.idx_var);

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

        llvm::Value *val_op
            = create(lhs,
                     rhs,
                     lhs_value,
                     rhs_value);

        // FIXME - CHARACTER assignment!
        llvm_visitor->ir_builder->CreateStore(
            val_op,
            llvm_visitor->ir_builder->CreateGEP(
                result_addr,
                { llvm_visitor->ir_builder->CreateLoad(result_idx_addr) }));

        llvm_visitor->ir_builder->CreateStore(
            llvm_visitor->ir_builder->CreateAdd(llvm_visitor->ir_builder->CreateLoad(result_idx_addr),
                                    llvm_visitor->get_integer_value_64(1)),
            result_idx_addr);
        create_loop_footer_for_array_op(node_type, loop_info_op);

        // The result array is an address in LLVM world.
        value = result_addr;
    }

    template <typename Node, typename Create>
    void binary_operator(const Node node,
                               Create create)
    {
        FortranLLVM::TrackLocation loc(llvm_visitor, node);

        Nodecl::NodeclBase lhs = node.get_lhs();
        Nodecl::NodeclBase rhs = node.get_rhs();

        TL::Type lhs_type = lhs.get_type();
        TL::Type rhs_type = rhs.get_type();

        if (lhs_type.is_fortran_array() || rhs_type.is_fortran_array())
        {
            return arithmetic_binary_operator_array(
                node, lhs, rhs, create);
        }

        // A scalar intrinsic binary operation
        llvm::Value *vlhs = llvm_visitor->eval_expression(lhs);
        llvm::Value *vrhs = llvm_visitor->eval_expression(rhs);

        value = arithmetic_binary_op_elemental_intrinsic(
            lhs, rhs, vlhs, vrhs, create);
    }

    llvm::Value *create_complex_value(TL::Type complex_type, llvm::Value *real, llvm::Value *imag)
    {
        llvm::Value *result = llvm::UndefValue::get(llvm_visitor->get_llvm_type(complex_type));
        result = llvm_visitor->ir_builder->CreateInsertElement(result, real, uint64_t(0));
        result = llvm_visitor->ir_builder->CreateInsertElement(result, imag, 1);
        return result;
    }

#define CREATOR(Creator) [&, this](Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value* lhs, llvm::Value* rhs) \
            { return llvm_visitor->ir_builder->Creator(lhs, rhs); }
#define INVALID_OP [&, this](Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value* lhs, llvm::Value* rhs) -> llvm::Value* \
            { internal_error("Invalid operation", 0); return nullptr; }
#define UNIMPLEMENTED_OP [&, this](Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value* lhs, llvm::Value* rhs) -> llvm::Value* \
            { internal_error("Operation not yet implemented", 0); return nullptr; }
    typedef std::function<llvm::Value*(Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value*, llvm::Value*)> BinaryOpCreator;
    BinaryOpCreator choose_arithmetic_creator(TL::Type t, 
           BinaryOpCreator create_integer,
           BinaryOpCreator create_real,
           BinaryOpCreator create_complex)
    {
        t = t.no_ref();
        if (t.is_fortran_array())
            t = t.array_base_element();
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
    void arithmetic_binary_operator(Node node,
           BinaryOpCreator create_integer,
           BinaryOpCreator create_real,
           BinaryOpCreator create_complex)
    {
        binary_operator(node,
            choose_arithmetic_creator(node.get_type(),
                create_integer, create_real, create_complex));
    }

    void visit(const Nodecl::Add& node)
    {
        arithmetic_binary_operator(node,
                              CREATOR(CreateAdd),
                              CREATOR(CreateFAdd),
                              CREATOR(CreateFAdd));
    }

    void visit(const Nodecl::Minus& node)
    {
        arithmetic_binary_operator(node,
                              CREATOR(CreateSub),
                              CREATOR(CreateFSub),
                              CREATOR(CreateFSub));
    }

    void visit(const Nodecl::Mul& node)
    {
        auto create_complex_mul = [&, this](Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value *lhs, llvm::Value *rhs) {
            // (a, b) * (c, d) = (ac - bd, ad + bc)
            llvm::Value *a = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(0));
            llvm::Value *b = llvm_visitor->ir_builder->CreateExtractElement(lhs, 1);
            llvm::Value *c = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(0));
            llvm::Value *d = llvm_visitor->ir_builder->CreateExtractElement(rhs, 1);

            llvm::Value *real_part = llvm_visitor->ir_builder->CreateFSub(
                llvm_visitor->ir_builder->CreateFMul(a, c),
                llvm_visitor->ir_builder->CreateFMul(b, d));
            llvm::Value *imag_part = llvm_visitor->ir_builder->CreateFAdd(
                llvm_visitor->ir_builder->CreateFMul(a, d),
                llvm_visitor->ir_builder->CreateFMul(b, c));

            return create_complex_value(node.get_type(), real_part, imag_part);
        };

        arithmetic_binary_operator(node,
                              CREATOR(CreateMul),
                              CREATOR(CreateFMul),
                              create_complex_mul);
    }

    void visit(const Nodecl::Div& node)
    {
        auto create_complex_div = [&, this](Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value *lhs, llvm::Value *rhs) {
            // (a, b) / (c, d) = ((ac + bd) / (c^2 + d^2), (bc - ad) / (c^2 + d^2))
            llvm::Value *a = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(0));
            llvm::Value *b = llvm_visitor->ir_builder->CreateExtractElement(lhs, 1);
            llvm::Value *c = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(0));
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
        arithmetic_binary_operator(node,
                              CREATOR(CreateSDiv),
                              CREATOR(CreateFDiv),
                              create_complex_div);
    }

    void visit(const Nodecl::Power& node)
    {
        // We use exponentiation by squaring
        // a ** b is either   a ** (2 * b) == (a*a) ** b
        //           or       a ** (2 * b + 1) == a * (a*a) ** b 
        auto create_pow_int = [&, this](Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value* lhs, llvm::Value *rhs) {
            llvm::BasicBlock *pow_loop_check = llvm::BasicBlock::Create(
                    llvm_visitor->llvm_context, "pow.loop.check", llvm_visitor->get_current_function());
            llvm::BasicBlock *pow_loop_body = llvm::BasicBlock::Create(
                    llvm_visitor->llvm_context, "pow.loop.body", llvm_visitor->get_current_function());
            llvm::BasicBlock *pow_loop_body_odd = llvm::BasicBlock::Create(
                    llvm_visitor->llvm_context, "pow.loop.body.odd", llvm_visitor->get_current_function());
            llvm::BasicBlock *pow_loop_body_common = llvm::BasicBlock::Create(
                    llvm_visitor->llvm_context, "pow.loop.body.common", llvm_visitor->get_current_function());
            llvm::BasicBlock *pow_loop_end = llvm::BasicBlock::Create(
                    llvm_visitor->llvm_context, "pow.loop.end", llvm_visitor->get_current_function());
            llvm::BasicBlock *pow_if_neg = llvm::BasicBlock::Create(
                    llvm_visitor->llvm_context, "pow.if.neg", llvm_visitor->get_current_function());
            llvm::BasicBlock *pow_if_end = llvm::BasicBlock::Create(
                    llvm_visitor->llvm_context, "pow.if.end", llvm_visitor->get_current_function());

            llvm::Value *result = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->get_llvm_type(node.get_type()));
            llvm::Value *base = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->get_llvm_type(node.get_type()));
            llvm::Value *exponent = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->get_llvm_type(node.get_type()));

            // rhs < 0
            llvm::Value *negative_exponent = llvm_visitor->ir_builder->CreateICmpSLT(rhs, 
                    llvm_visitor->get_integer_value(0, node.get_type()));

            // result <- 1
            llvm_visitor->ir_builder->CreateStore(
                    llvm_visitor->get_integer_value(1, node.get_type()),
                    result);
            // base <- lhs
            llvm_visitor->ir_builder->CreateStore(
                    lhs,
                    base);
            // exponent <- |rhs|
            llvm_visitor->ir_builder->CreateStore(
                    llvm_visitor->ir_builder->CreateSelect(
                        negative_exponent,
                        llvm_visitor->ir_builder->CreateSub(
                            llvm_visitor->get_integer_value(0, node.get_type()),
                            rhs),
                        rhs),
                    exponent);

            llvm_visitor->ir_builder->CreateBr(pow_loop_check);

            llvm_visitor->set_current_block(pow_loop_check);
            // exponent != 0 
            llvm::Value *cond_val = llvm_visitor->ir_builder->CreateICmpNE(
                    llvm_visitor->ir_builder->CreateLoad(exponent),
                    llvm_visitor->get_integer_value(0, node.get_type()));
            llvm_visitor->ir_builder->CreateCondBr(cond_val, pow_loop_body, pow_loop_end);

            llvm_visitor->set_current_block(pow_loop_body);
            // exponent % 2
            cond_val = llvm_visitor->ir_builder->CreateZExtOrTrunc(
                    llvm_visitor->ir_builder->CreateSRem(
                        llvm_visitor->ir_builder->CreateLoad(exponent),
                        llvm_visitor->get_integer_value(2, node.get_type())),
                    llvm_visitor->llvm_types.i1);
            llvm_visitor->ir_builder->CreateCondBr(cond_val, pow_loop_body_odd, pow_loop_body_common);

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
                    llvm_visitor->ir_builder->CreateMul(
                        base_load,
                        base_load),
                    base);

            llvm_visitor->ir_builder->CreateBr(pow_loop_check);

            llvm_visitor->set_current_block(pow_loop_end);
            llvm_visitor->ir_builder->CreateCondBr(negative_exponent,
                pow_if_neg,
                pow_if_end);

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

        auto create_pow_float = [&, this](Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value* lhs, llvm::Value *rhs) {
            llvm::Function *powf = llvm::Intrinsic::getDeclaration(
                    llvm_visitor->current_module.get(),
                    llvm::Intrinsic::pow,
                    {lhs->getType(), rhs->getType()});
            ERROR_CONDITION(powf == nullptr, "llvm.pow not found?", 0);
            return llvm_visitor->ir_builder->CreateCall(powf, {lhs, rhs});
        };

        auto create_pow_complex = [&, this](Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value* lhs, llvm::Value *rhs) {
            llvm::Type* complex_type = llvm_visitor->get_llvm_type(node.get_type());
            std::string cpow_name;
            TL::Type base_type = node.get_type().complex_get_base_type();
            if (base_type.is_float())
                cpow_name = "cpowf";
            else if (base_type.is_double())
                cpow_name = "cpow";
            else
                internal_error("Unsupported type for complex power '%s'\n", print_declarator(node.get_type().get_internal_type()));

            llvm::Function *cpow_fun = llvm::cast<llvm::Function>(
                    llvm_visitor->current_module->getOrInsertFunction(
                        cpow_name,
                        llvm::FunctionType::get(
                            complex_type,
                            { complex_type, complex_type },
                            /* isVarArg */ false),
                        /* no attributes so far */ llvm::AttributeSet()));
            return llvm_visitor->ir_builder->CreateCall(cpow_fun, {lhs, rhs});
        };

        arithmetic_binary_operator(node,
                create_pow_int,
                create_pow_float,
                create_pow_complex);
    }

    template <typename Node, typename CreateSInt, typename CreateFloat, typename CreateComplex, typename CreateCharacter>
    void arithmetic_binary_comparison(const Node node,
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
            internal_error("Unexpected type '%s' and '%s' for arithmetic binary relational operator\n",
                    print_declarator(lhs_type.get_internal_type()),
                    print_declarator(rhs_type.get_internal_type()));

        binary_operator(node, create);

        // Make sure the logical value stays in the proper integer size
        // and not just i1.
        // Note the usage of CreateZExtOrTrunc here instead of the usual SExt
        // otherwise the resulting value would be 0 or -1 and we want 0 or 1
        value = llvm_visitor->ir_builder->CreateZExtOrTrunc(
            value, llvm_visitor->get_llvm_type(node.get_type()));
    }

    struct CharacterCompareLT
    {
        private:
            FortranLLVM* llvm_visitor;

        public:
            CharacterCompareLT(FortranLLVM* llvm_visitor) : llvm_visitor(llvm_visitor) { }

            llvm::Value* operator()(
                    Nodecl::NodeclBase lhs, Nodecl::NodeclBase rhs,
                    llvm::Value *vlhs, llvm::Value *vrhs)
            {
                llvm::Value *len_lhs = llvm_visitor->eval_length_of_character(lhs.get_type());
                llvm::Value *len_rhs = llvm_visitor->eval_length_of_character(rhs.get_type());

                llvm::Value *len_min = llvm_visitor->ir_builder->CreateSelect(
                        llvm_visitor->ir_builder->CreateICmpSLT(len_lhs, len_rhs),
                        len_lhs, len_rhs);

                llvm::BasicBlock *loop_check = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.check", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_body = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.body", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_body_check = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.body.check", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_body_next = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.body.next", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_end = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.end", llvm_visitor->get_current_function());

                llvm::BasicBlock *tail_check = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.loop.check", llvm_visitor->get_current_function());
                llvm::BasicBlock *tail_body = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.loop.body", llvm_visitor->get_current_function());
                llvm::BasicBlock *tail_body_next = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.loop.body.next", llvm_visitor->get_current_function());
                llvm::BasicBlock *tail_nonempty = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.nonempty", llvm_visitor->get_current_function());

                llvm::BasicBlock *cmp_true = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.true", llvm_visitor->get_current_function());
                llvm::BasicBlock *cmp_false = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.false", llvm_visitor->get_current_function());
                llvm::BasicBlock *cmp_end = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.end", llvm_visitor->get_current_function());

                // Normalize operands types
                vlhs = llvm_visitor->ir_builder->CreatePointerCast(vlhs, llvm_visitor->llvm_types.ptr_i8);
                vrhs = llvm_visitor->ir_builder->CreatePointerCast(vrhs, llvm_visitor->llvm_types.ptr_i8);

                // A first loop does the comparison between the prefix
                llvm::Value *idx_var = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
                llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0), idx_var);
                llvm_visitor->ir_builder->CreateBr(loop_check);

                llvm_visitor->set_current_block(loop_check);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            llvm_visitor->ir_builder->CreateLoad(idx_var),
                            len_min),
                        loop_body,
                        loop_end);

                llvm_visitor->set_current_block(loop_body);

                llvm::Value *lhs_offset = llvm_visitor->ir_builder->CreateGEP(
                        vlhs,
                        { llvm_visitor->ir_builder->CreateLoad(idx_var) });
                llvm::Value *lhs_char = llvm_visitor->ir_builder->CreateLoad(lhs_offset);

                llvm::Value *rhs_offset = llvm_visitor->ir_builder->CreateGEP(
                        vrhs,
                        { llvm_visitor->ir_builder->CreateLoad(idx_var) });
                llvm::Value *rhs_char = llvm_visitor->ir_builder->CreateLoad(rhs_offset);

                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpEQ(
                            lhs_char,
                            rhs_char),
                        loop_body_next,
                        loop_body_check);

                llvm_visitor->set_current_block(loop_body_check);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            lhs_char,
                            rhs_char),
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

                // A second loop makes sure that the remainder of the longest string is all blanks
                llvm::Value *checked_character = llvm_visitor->ir_builder->CreateSelect(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            llvm_visitor->ir_builder->CreateLoad(idx_var),
                            len_lhs),
                        vlhs,
                        vrhs);
                llvm::Value *len_max = llvm_visitor->ir_builder->CreateSelect(
                        llvm_visitor->ir_builder->CreateICmpSGT(len_lhs, len_rhs),
                        len_lhs, len_rhs);
                llvm_visitor->ir_builder->CreateBr(tail_check);

                llvm_visitor->set_current_block(tail_check);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            llvm_visitor->ir_builder->CreateLoad(idx_var),
                            len_max),
                        tail_body,
                        cmp_false);

                llvm_visitor->set_current_block(tail_body);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpEQ(
                            llvm_visitor->ir_builder->CreateLoad(
                                llvm_visitor->ir_builder->CreateGEP(
                                    llvm_visitor->ir_builder->CreatePointerCast(checked_character, llvm_visitor->llvm_types.ptr_i8),
                                    { llvm_visitor->ir_builder->CreateLoad(idx_var) })),
                            llvm_visitor->get_integer_value_N(' ', llvm_visitor->llvm_types.i8, 8)),
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
                        cmp_true, cmp_false);

                llvm_visitor->set_current_block(cmp_true);
                llvm::Value *result_true = llvm_visitor->get_integer_value_N(1, llvm_visitor->llvm_types.i1, 1);
                llvm_visitor->ir_builder->CreateBr(cmp_end);

                llvm_visitor->set_current_block(cmp_false);
                llvm::Value *result_false = llvm_visitor->get_integer_value_N(0, llvm_visitor->llvm_types.i1, 1);
                llvm_visitor->ir_builder->CreateBr(cmp_end);

                llvm_visitor->set_current_block(cmp_end);
                llvm::Value *value = llvm_visitor->ir_builder->CreatePHI(
                        llvm_visitor->llvm_types.i1, 2);
                llvm::cast<llvm::PHINode>(value)->addIncoming(result_true, cmp_true);
                llvm::cast<llvm::PHINode>(value)->addIncoming(result_false, cmp_false);

                return value;
            }
    };

    void visit(const Nodecl::LowerThan &node)
    {
        CharacterCompareLT character_compare_lt(llvm_visitor);
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpSLT),
                              CREATOR(CreateFCmpOLT),
                              INVALID_OP,
                              character_compare_lt);
    }

    struct CharacterCompareLE
    {
        private:
            FortranLLVM* llvm_visitor;

        public:
            CharacterCompareLE(FortranLLVM* llvm_visitor) : llvm_visitor(llvm_visitor) { }

            llvm::Value* operator()(
                    Nodecl::NodeclBase lhs, Nodecl::NodeclBase rhs,
                    llvm::Value *vlhs, llvm::Value *vrhs)
            {
                llvm::Value *len_lhs = llvm_visitor->eval_length_of_character(lhs.get_type());
                llvm::Value *len_rhs = llvm_visitor->eval_length_of_character(rhs.get_type());

                llvm::Value *len_min = llvm_visitor->ir_builder->CreateSelect(
                        llvm_visitor->ir_builder->CreateICmpSLT(len_lhs, len_rhs),
                        len_lhs, len_rhs);

                llvm::BasicBlock *loop_check = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.check", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_body = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.body", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_body_check = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.body.check", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_body_next = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.body.next", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_end = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.end", llvm_visitor->get_current_function());

                llvm::BasicBlock *tail_check = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.loop.check", llvm_visitor->get_current_function());
                llvm::BasicBlock *tail_body = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.loop.body", llvm_visitor->get_current_function());
                llvm::BasicBlock *tail_body_next = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.loop.body.next", llvm_visitor->get_current_function());
                llvm::BasicBlock *tail_nonempty = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.nonempty", llvm_visitor->get_current_function());

                llvm::BasicBlock *cmp_true = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.true", llvm_visitor->get_current_function());
                llvm::BasicBlock *cmp_false = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.false", llvm_visitor->get_current_function());
                llvm::BasicBlock *cmp_end = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.end", llvm_visitor->get_current_function());

                // Normalize operands types
                vlhs = llvm_visitor->ir_builder->CreatePointerCast(vlhs, llvm_visitor->llvm_types.ptr_i8);
                vrhs = llvm_visitor->ir_builder->CreatePointerCast(vrhs, llvm_visitor->llvm_types.ptr_i8);

                // A first loop does the comparison between the prefix
                llvm::Value *idx_var = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
                llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0), idx_var);
                llvm_visitor->ir_builder->CreateBr(loop_check);

                llvm_visitor->set_current_block(loop_check);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            llvm_visitor->ir_builder->CreateLoad(idx_var),
                            len_min),
                        loop_body,
                        loop_end);

                llvm_visitor->set_current_block(loop_body);

                llvm::Value *lhs_offset = llvm_visitor->ir_builder->CreateGEP(
                        vlhs,
                        { llvm_visitor->ir_builder->CreateLoad(idx_var) });
                llvm::Value *lhs_char = llvm_visitor->ir_builder->CreateLoad(lhs_offset);

                llvm::Value *rhs_offset = llvm_visitor->ir_builder->CreateGEP(
                        vrhs,
                        { llvm_visitor->ir_builder->CreateLoad(idx_var) });
                llvm::Value *rhs_char = llvm_visitor->ir_builder->CreateLoad(rhs_offset);

                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpEQ(
                            lhs_char,
                            rhs_char),
                        loop_body_next,
                        loop_body_check);

                llvm_visitor->set_current_block(loop_body_check);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpSLE(
                            lhs_char,
                            rhs_char),
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

                // A second loop makes sure that the remainder of the longest string is all blanks
                llvm::Value *checked_character = llvm_visitor->ir_builder->CreateSelect(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            llvm_visitor->ir_builder->CreateLoad(idx_var),
                            len_lhs),
                        vlhs,
                        vrhs);
                llvm::Value *len_max = llvm_visitor->ir_builder->CreateSelect(
                        llvm_visitor->ir_builder->CreateICmpSGT(len_lhs, len_rhs),
                        len_lhs, len_rhs);
                llvm_visitor->ir_builder->CreateBr(tail_check);

                llvm_visitor->set_current_block(tail_check);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            llvm_visitor->ir_builder->CreateLoad(idx_var),
                            len_max),
                        tail_body,
                        cmp_true);

                llvm_visitor->set_current_block(tail_body);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpEQ(
                            llvm_visitor->ir_builder->CreateLoad(
                                llvm_visitor->ir_builder->CreateGEP(
                                    llvm_visitor->ir_builder->CreatePointerCast(checked_character, llvm_visitor->llvm_types.ptr_i8),
                                    { llvm_visitor->ir_builder->CreateLoad(idx_var) })),
                            llvm_visitor->get_integer_value_N(' ', llvm_visitor->llvm_types.i8, 8)),
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
                        cmp_true, cmp_false);

                llvm_visitor->set_current_block(cmp_true);
                llvm::Value *result_true = llvm_visitor->get_integer_value_N(1, llvm_visitor->llvm_types.i1, 1);
                llvm_visitor->ir_builder->CreateBr(cmp_end);

                llvm_visitor->set_current_block(cmp_false);
                llvm::Value *result_false = llvm_visitor->get_integer_value_N(0, llvm_visitor->llvm_types.i1, 1);
                llvm_visitor->ir_builder->CreateBr(cmp_end);

                llvm_visitor->set_current_block(cmp_end);
                llvm::Value *value = llvm_visitor->ir_builder->CreatePHI(
                        llvm_visitor->llvm_types.i1, 2);
                llvm::cast<llvm::PHINode>(value)->addIncoming(result_true, cmp_true);
                llvm::cast<llvm::PHINode>(value)->addIncoming(result_false, cmp_false);

                return value;
            }
    };

    void visit(const Nodecl::LowerOrEqualThan &node)
    {
        CharacterCompareLE character_compare_le(llvm_visitor);
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpSLE),
                              CREATOR(CreateFCmpOLE),
                              INVALID_OP,
                              character_compare_le);
    }

    void visit(const Nodecl::GreaterThan &node)
    {
        auto character_compare_gt = [&, this](Nodecl::NodeclBase lhs, Nodecl::NodeclBase rhs,
                llvm::Value* vlhs, llvm::Value *vrhs)
        {
            CharacterCompareLT character_compare_lt(llvm_visitor);
            // a > b computed as b < a
            return character_compare_lt(rhs, lhs, vrhs, vlhs);
        };
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpSGT),
                              CREATOR(CreateFCmpOGT),
                              INVALID_OP,
                              character_compare_gt);
    }

    void visit(const Nodecl::GreaterOrEqualThan &node)
    {
        auto character_compare_ge = [&, this](Nodecl::NodeclBase lhs, Nodecl::NodeclBase rhs,
                llvm::Value* vlhs, llvm::Value *vrhs) -> llvm::Value*
        {
            CharacterCompareLE character_compare_le(llvm_visitor);
            // a >= b computed as b <= a
            return character_compare_le(rhs, lhs, vrhs, vlhs);
        };
        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpSGE),
                              CREATOR(CreateFCmpOGE),
                              INVALID_OP,
                              character_compare_ge);
    }

    struct CharacterCompareEQ
    {
        private:
            FortranLLVM* llvm_visitor;

        public:
            CharacterCompareEQ(FortranLLVM* llvm_visitor) : llvm_visitor(llvm_visitor) { }

            llvm::Value* operator()(
                    Nodecl::NodeclBase lhs, Nodecl::NodeclBase rhs,
                    llvm::Value *vlhs, llvm::Value *vrhs)
            {
                llvm::Value *len_lhs = llvm_visitor->eval_length_of_character(lhs.get_type());
                llvm::Value *len_rhs = llvm_visitor->eval_length_of_character(rhs.get_type());

                llvm::Value *len_min = llvm_visitor->ir_builder->CreateSelect(
                        llvm_visitor->ir_builder->CreateICmpSLT(len_lhs, len_rhs),
                        len_lhs, len_rhs);

                llvm::BasicBlock *loop_check = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.check", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_body = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.body", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_body_next = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.body.next", llvm_visitor->get_current_function());
                llvm::BasicBlock *loop_end = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.loop.end", llvm_visitor->get_current_function());

                llvm::BasicBlock *tail_check = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.loop.check", llvm_visitor->get_current_function());
                llvm::BasicBlock *tail_body = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.loop.body", llvm_visitor->get_current_function());
                llvm::BasicBlock *tail_body_next = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.tail.loop.body.next", llvm_visitor->get_current_function());

                llvm::BasicBlock *cmp_true = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.true", llvm_visitor->get_current_function());
                llvm::BasicBlock *cmp_false = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.false", llvm_visitor->get_current_function());
                llvm::BasicBlock *cmp_end = llvm::BasicBlock::Create(llvm_visitor->llvm_context, "character.cmp.end", llvm_visitor->get_current_function());

                // Normalize operands types
                vlhs = llvm_visitor->ir_builder->CreatePointerCast(vlhs, llvm_visitor->llvm_types.ptr_i8);
                vrhs = llvm_visitor->ir_builder->CreatePointerCast(vrhs, llvm_visitor->llvm_types.ptr_i8);

                // A first loop does the comparison between the prefix
                llvm::Value *idx_var = llvm_visitor->ir_builder->CreateAlloca(llvm_visitor->llvm_types.i64);
                llvm_visitor->ir_builder->CreateStore(llvm_visitor->get_integer_value_64(0), idx_var);
                llvm_visitor->ir_builder->CreateBr(loop_check);

                llvm_visitor->set_current_block(loop_check);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            llvm_visitor->ir_builder->CreateLoad(idx_var),
                            len_min),
                        loop_body,
                        loop_end);

                llvm_visitor->set_current_block(loop_body);

                llvm::Value *lhs_offset = llvm_visitor->ir_builder->CreateGEP(
                        vlhs,
                        { llvm_visitor->ir_builder->CreateLoad(idx_var) });
                llvm::Value *lhs_char = llvm_visitor->ir_builder->CreateLoad(lhs_offset);

                llvm::Value *rhs_offset = llvm_visitor->ir_builder->CreateGEP(
                        vrhs,
                        { llvm_visitor->ir_builder->CreateLoad(idx_var) });
                llvm::Value *rhs_char = llvm_visitor->ir_builder->CreateLoad(rhs_offset);

                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpEQ(
                            lhs_char,
                            rhs_char),
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

                // A second loop makes sure that the remainder of the longest string is all blanks
                llvm::Value *checked_character = llvm_visitor->ir_builder->CreateSelect(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            llvm_visitor->ir_builder->CreateLoad(idx_var),
                            len_lhs),
                        vlhs,
                        vrhs);
                llvm::Value *len_max = llvm_visitor->ir_builder->CreateSelect(
                        llvm_visitor->ir_builder->CreateICmpSGT(len_lhs, len_rhs),
                        len_lhs, len_rhs);
                llvm_visitor->ir_builder->CreateBr(tail_check);

                llvm_visitor->set_current_block(tail_check);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpSLT(
                            llvm_visitor->ir_builder->CreateLoad(idx_var),
                            len_max),
                        tail_body,
                        cmp_true);

                llvm_visitor->set_current_block(tail_body);
                llvm_visitor->ir_builder->CreateCondBr(
                        llvm_visitor->ir_builder->CreateICmpEQ(
                            llvm_visitor->ir_builder->CreateLoad(
                                llvm_visitor->ir_builder->CreateGEP(
                                    llvm_visitor->ir_builder->CreatePointerCast(checked_character, llvm_visitor->llvm_types.ptr_i8),
                                    { llvm_visitor->ir_builder->CreateLoad(idx_var) })),
                            llvm_visitor->get_integer_value_N(' ', llvm_visitor->llvm_types.i8, 8)),
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
                llvm::Value *result_true = llvm_visitor->get_integer_value_N(1, llvm_visitor->llvm_types.i1, 1);
                llvm_visitor->ir_builder->CreateBr(cmp_end);

                llvm_visitor->set_current_block(cmp_false);
                llvm::Value *result_false = llvm_visitor->get_integer_value_N(0, llvm_visitor->llvm_types.i1, 1);
                llvm_visitor->ir_builder->CreateBr(cmp_end);

                llvm_visitor->set_current_block(cmp_end);
                llvm::Value *value = llvm_visitor->ir_builder->CreatePHI(
                        llvm_visitor->llvm_types.i1, 2);
                llvm::cast<llvm::PHINode>(value)->addIncoming(result_true, cmp_true);
                llvm::cast<llvm::PHINode>(value)->addIncoming(result_false, cmp_false);

                return value;
            }
    };

    void visit(const Nodecl::Equal &node)
    {
        auto creator_complex = [&, this](Nodecl::NodeclBase,
            Nodecl::NodeclBase,
            llvm::Value *lhs,
            llvm::Value *rhs) -> llvm::Value*
        {
            llvm::Value *lhs_real = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(0));
            llvm::Value *lhs_imag = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(1));

            llvm::Value *rhs_real = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(0));
            llvm::Value *rhs_imag = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(1));

            return llvm_visitor->ir_builder->CreateSelect(llvm_visitor->ir_builder->CreateFCmpOEQ(lhs_real, rhs_real),
                    llvm_visitor->ir_builder->CreateFCmpOEQ(lhs_imag, rhs_imag),
                    llvm_visitor->get_integer_value_N(0, llvm_visitor->llvm_types.i1, 1));
        };

        CharacterCompareEQ character_compare_eq(llvm_visitor);

        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpEQ),
                              CREATOR(CreateFCmpOEQ),
                              creator_complex,
                              character_compare_eq);
    }

    void visit(const Nodecl::Different &node)
    {
        auto creator_complex = [&, this](Nodecl::NodeclBase,
            Nodecl::NodeclBase,
            llvm::Value *lhs,
            llvm::Value *rhs) -> llvm::Value*
        {
            llvm::Value *lhs_real = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(0));
            llvm::Value *lhs_imag = llvm_visitor->ir_builder->CreateExtractElement(lhs, uint64_t(1));

            llvm::Value *rhs_real = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(0));
            llvm::Value *rhs_imag = llvm_visitor->ir_builder->CreateExtractElement(rhs, uint64_t(1));

            return llvm_visitor->ir_builder->CreateSelect(llvm_visitor->ir_builder->CreateFCmpONE(lhs_real, rhs_real),
                    llvm_visitor->ir_builder->CreateFCmpONE(lhs_imag, rhs_imag),
                    llvm_visitor->get_integer_value_N(0, llvm_visitor->llvm_types.i1, 1));
        };

        auto character_compare_ne = [&, this](Nodecl::NodeclBase lhs, Nodecl::NodeclBase rhs,
            llvm::Value *vlhs, llvm::Value *vrhs) -> llvm::Value*
        {
            CharacterCompareEQ compare_eq(llvm_visitor);
            llvm::Value *v = llvm_visitor->ir_builder->CreateSelect(
                compare_eq(lhs, rhs, vlhs, vrhs),
                llvm_visitor->get_integer_value_N(0, llvm_visitor->llvm_types.i1, 1),
                llvm_visitor->get_integer_value_N(1, llvm_visitor->llvm_types.i1, 1));
            return v;
        };

        arithmetic_binary_comparison(node,
                              CREATOR(CreateICmpNE),
                              CREATOR(CreateFCmpONE),
                              creator_complex,
                              character_compare_ne);
    }
#undef CREATOR
#undef UNIMPLEMENTED_OP
#undef INVALID_OP

    void visit(const Nodecl::Concat& node)
    {
        auto create_concat = [&, this](Nodecl::NodeclBase lhs, Nodecl::NodeclBase rhs,
                                       llvm::Value *vlhs, llvm::Value *vrhs) -> llvm::Value* {
            TL::Type lhs_type = lhs.get_type();
            while (lhs_type.is_fortran_array())
                lhs_type = lhs_type.array_element();

            // TODO - CHARACTER(LEN=*) is special as it uses a hidden parameter
            llvm::Value *lhs_elements = llvm_visitor->eval_length_of_character(lhs_type);

            TL::Type rhs_type = rhs.get_type();
            while (rhs_type.is_fortran_array())
                rhs_type = rhs_type.array_element();

            llvm::Value *rhs_elements = llvm_visitor->eval_length_of_character(rhs_type);

            llvm::Value *total_elements = llvm_visitor->ir_builder->CreateAdd(lhs_elements, rhs_elements);

            // We only support CHARACTER(KIND=1) so i8 is fine here
            llvm::Value *result_addr = llvm_visitor->ir_builder->CreateAlloca(
                    llvm_visitor->llvm_types.i8, total_elements);

            llvm_visitor->ir_builder->CreateMemCpy(
                result_addr,
                vlhs,
                lhs_elements,
                1);

            llvm::Value *rhs_offset = llvm_visitor->ir_builder->CreateGEP(result_addr, { lhs_elements });

            llvm_visitor->ir_builder->CreateMemCpy(
                rhs_offset,
                vrhs,
                rhs_elements,
                1);

            return result_addr;
        };
        binary_operator(node, create_concat);
    }
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
        FortranLLVM::TrackLocation loc(llvm_visitor, node);

        value = llvm_visitor->get_integer_value(
            // FIXME: INTEGER(16)
            const_value_cast_to_8(node.get_constant()),
            node.get_type());
    }

    void visit(const Nodecl::FloatingLiteral& node)
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
    void visit(const Nodecl::ParenthesizedExpression& node)
    {
        FortranLLVM::TrackLocation loc(llvm_visitor, node);

        // FIXME: Make sure the code preserves the integrity of the parentheses
        // (i.e. (A + B) + C does not become A + (B + C)
        walk(node.get_nest());
    }

    // FIXME - Use GEP when possible
    llvm::Value *address_of_subscripted_array_no_descriptor(
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
            llvm::Value *val_lower
                = llvm_visitor->ir_builder->CreateSExtOrTrunc(
                    llvm_visitor->eval_expression(lower),
                    llvm_visitor->llvm_types.i64);
            llvm::Value *val_upper
                = llvm_visitor->ir_builder->CreateSExtOrTrunc(
                    llvm_visitor->eval_expression(upper),
                    llvm_visitor->llvm_types.i64);

            ERROR_CONDITION(
                index.is<Nodecl::Range>(), "Invalid subscript here", 0);
            llvm::Value *val_idx = llvm_visitor->ir_builder->CreateSExtOrTrunc(
                llvm_visitor->eval_expression(index),
                llvm_visitor->llvm_types.i64);

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

        llvm::Value *base_addr
            = llvm_visitor->eval_expression(subscripted);

        return llvm_visitor->ir_builder->CreateGEP(
                base_addr,
                { val_addr });
    }

    llvm::Value *address_of_subscripted_array_descriptor(
        const Nodecl::ArraySubscript &node)
    {
        Nodecl::NodeclBase subscripted = node.get_subscripted();
        TL::Type subscripted_type = subscripted.get_type();
        TL::Type subscripted_type_noref = subscripted_type.no_ref();

        llvm::Value *descriptor_addr
            = llvm_visitor->eval_expression(subscripted);

        Nodecl::List subscripts_tree = node.get_subscripts().as<Nodecl::List>();
        // Reverse subscripts as in the tree are represented in the C order
        TL::ObjectList<Nodecl::NodeclBase> subscripts(subscripts_tree.rbegin(),
                                                      subscripts_tree.rend());

        llvm::Value *linear_index = nullptr;
        int rank = 0;
        for (TL::ObjectList<Nodecl::NodeclBase>::iterator it
             = subscripts.begin();
             it != subscripts.end();
             it++, rank++)
        {
            llvm::Value *current = llvm_visitor->ir_builder->CreateMul(
                llvm_visitor->ir_builder->CreateZExtOrTrunc(
                    llvm_visitor->eval_expression(*it),
                    llvm_visitor->llvm_types.i64),
                llvm_visitor->ir_builder->CreateLoad(
                    llvm_visitor->array_descriptor_addr_dim_stride(
                        descriptor_addr, rank)));

            if (linear_index == nullptr)
                linear_index = current;
            else
                linear_index = llvm_visitor->ir_builder->CreateAdd(linear_index,
                                                                   current);
        }

        llvm::Value *offset_value = llvm_visitor->ir_builder->CreateLoad(
            llvm_visitor->array_descriptor_addr_offset(descriptor_addr));

        linear_index
            = llvm_visitor->ir_builder->CreateAdd(linear_index, offset_value);

        TL::Type element_type
            = subscripted_type_noref.fortran_array_base_element();

        llvm::Value *base_address = llvm_visitor->ir_builder->CreateLoad(
            llvm_visitor->array_descriptor_addr_base_addr(descriptor_addr));
        base_address = llvm_visitor->ir_builder->CreatePointerCast(
            base_address,
            llvm::PointerType::get(llvm_visitor->get_llvm_type(element_type),
                                   /* AddressSpace */ 0));

        return llvm_visitor->ir_builder->CreateGEP(base_address,
                                                   { linear_index });
    }

    void visit(const Nodecl::ArraySubscript& node)
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

    void visit(const Nodecl::FunctionCall& node)
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

        bool call_without_interface = called_type.lacks_prototype()
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
                && node.get_alternate_name().get_symbol().get_type().lacks_prototype());

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
                ret_type = node.get_alternate_name().get_symbol().get_type().returns();
            }
            // Note, this will create a function with int parameters. Make sure they are not used.
            called_type = ::get_nonproto_function_type(
                    ret_type.get_internal_type(),
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
                        varg,
                        llvm_visitor->llvm_types.ptr_i8);
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
                    else if (!arg.get_type()
                                  .no_ref()
                                  .array_requires_descriptor()
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
                            arg.get_type().no_ref(),
                            descriptor_addr,
                            base_address);

                        varg = descriptor_addr;
                    }
                    else if (arg.get_type()
                                  .no_ref()
                                  .array_requires_descriptor()
                             && !it_param->no_ref().array_requires_descriptor())
                    {
                        // If the array is CONTIGUOUS we can use the buffer
                        // directly, otherwise we will need to create a
                        // temporary
                        internal_error("Not yet implemented", 0);
                    }
                    else if (arg.get_type()
                                  .no_ref()
                                  .array_requires_descriptor()
                             && it_param->no_ref().array_requires_descriptor())
                    {
                        // Nothing special is required, we already have a proper descriptor
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
        ERROR_CONDITION(it_arg != arguments.end()
                            || it_param != parameters.end(),
                        "Mismatch between arguments and parameters",
                        0);

        if (call_without_interface)
        {
            llvm::Value *bitcast = llvm_visitor->ir_builder->CreateBitCast(
                    fun,
                    llvm::PointerType::get(function_type, /* AddressSpace */ 0));
            value = llvm_visitor->ir_builder->CreateCall(bitcast, val_arguments);
        }
        else
        {
            value = llvm_visitor->ir_builder->CreateCall(fun, val_arguments);
        }
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

  private:
    typedef void (FortranVisitorLLVMExpression::*BuiltinImplFunc)(
        const Nodecl::FunctionCall &node);

    struct BuiltinImpl
    {
        BuiltinImplFunc func = nullptr;
        bool use_constant = false;
    };

    // See the definition below
    static std::map<std::string, BuiltinImpl> builtin_impl;

    void implement_builtin_call(const Nodecl::FunctionCall &node)
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
            internal_error("No implementation for builtin '%s'\n", called_sym.get_name().c_str());
        }

        return (this->*(impl.func))(node);
    }

    void builtin_xbound(const Nodecl::FunctionCall &node, bool is_lbound)
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
        ERROR_CONDITION(dim_val > t.fortran_rank(), "Invalid dimension %d > %d", dim_val, t.fortran_rank());
        if (t.array_requires_descriptor())
        {
            llvm::Value *descriptor_addr = llvm_visitor->eval_expression(array);

            llvm::Value *xbound_field
                = is_lbound ?
                      llvm_visitor->array_descriptor_addr_dim_lower_bound(descriptor_addr,
                                                            dim_val - 1) :
                      llvm_visitor->array_descriptor_addr_dim_upper_bound(descriptor_addr,
                                                            dim_val - 1);

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

    void builtin_lbound(const Nodecl::FunctionCall &node)
    {
        return builtin_xbound(node, /* is_lbound */ true);
    }

    void builtin_ubound(const Nodecl::FunctionCall &node)
    {
        return builtin_xbound(node, /* is_lbound */ false);
    }
};

std::map<std::string, FortranVisitorLLVMExpression::BuiltinImpl>
    FortranVisitorLLVMExpression::builtin_impl
    = { { "lbound", { &FortranVisitorLLVMExpression::builtin_lbound, true } },
        { "ubound", { &FortranVisitorLLVMExpression::builtin_ubound, true } } };

llvm::Value *FortranLLVM::eval_constant(const_value_t *cval)
{
    if (const_value_is_integer(cval))
    {
        // FIXME: This should be elsewhere
        llvm::Type *t = nullptr;
        switch (const_value_get_bytes(cval))
        {
            case 1:
                t = llvm_types.i8;
            case 2:
                t = llvm_types.i16;
            case 4:
                t = llvm_types.i32;
            case 8:
                t = llvm_types.i64;
            default:
                internal_error("Code unreachable", 0);
        }
        return llvm::Constant::getIntegerValue(t,
                llvm::APInt(const_value_get_bytes(cval) * 8,
                    const_value_cast_to_8(cval),
                    const_value_is_signed(cval)));
    }
    else
    {
        internal_error("Constant evaluation of '%s' not implemented yet",
                       const_value_to_str(cval));
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
                FortranVisitorLLVMExpression::CharacterCompareEQ character_compare_eq(this);
                check = character_compare_eq(
                            current_switch().expr, current_case,
                            current_switch().value, current_value);
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

        if (t.lacks_prototype())
        {
            for (TL::Type t : params)
            {
                llvm_params.push_back(llvm_types.ptr_i8);
            }
        }
        else
        {
            for (TL::Type t : params)
            {
                llvm_params.push_back(get_llvm_type(t));
            }
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
    else if (t.is_char())
    {
        return llvm_types.i8;
    }
    else if (t.is_void())
    {
        return llvm_types.void_;
    }
    else if (t.is_complex())
    {
        llvm::Type *base_type = get_llvm_type(t.complex_get_base_type());
        // tuple: real, imag
        return llvm::VectorType::get(base_type, 2);
    }
    else if (t.is_fortran_character())
    {
        // FIXME - CHARACTER(LEN=*)
        Nodecl::NodeclBase size = t.array_get_size();
        ERROR_CONDITION(!size.is_constant(), "Invalid size", 0);

        return llvm::ArrayType::get(get_llvm_type(t.array_element()),
                                    const_value_cast_to_8(size.get_constant()));
    }
    else if (t.is_fortran_array()
            && !t.array_requires_descriptor())
    {
        // Use the base element type
        return get_llvm_type(t.array_base_element());
    }
    else if (t.is_fortran_array()
            && t.array_requires_descriptor())
    {
        return get_gfortran_array_descriptor_type(t);
    }
    else
    {
        internal_error("Cannot synthesize LLVM type for type '%s'",
                       print_declarator(t.get_internal_type()));
    }
}

llvm::DIType *FortranLLVM::get_debug_info_type(TL::Type t)
{
    if (t.is_lvalue_reference())
    {
        return dbg_builder->createReferenceType(llvm::dwarf::DW_TAG_reference_type,
                get_debug_info_type(t.references_to()),
                /* FIXME */ 8 * TL::Type::get_void_type().get_pointer_to().get_size());
    }
    // else if (t.is_pointer())
    // {
    //     return PointerType::get(
    //             get_debug_info_type(t.points_to()),
    //             /* AddressSpace */ 0);
    // }
    else if (t.is_function())
    {
        TL::ObjectList<TL::Type> params = t.parameters();
        std::vector<llvm::Metadata *> dbg_param_types;
        dbg_param_types.reserve(params.size() + 1);

        dbg_param_types.push_back(get_debug_info_type(t.returns()));
        for (TL::Type t : params)
        {
            dbg_param_types.push_back(get_debug_info_type(t));
        }
        return dbg_builder->createSubroutineType(dbg_builder->getOrCreateTypeArray(dbg_param_types));
    }
    else if (t.is_bool())
    {
        std::stringstream ss;
        ss << "LOGICAL(KIND=" << t.get_size() << ")";
        return dbg_builder->createBasicType(ss.str(), t.get_size() * 8, llvm::dwarf::DW_ATE_boolean);
    }
    else if (t.is_signed_integral())
    {
        std::stringstream ss;
        ss << "INTEGER(KIND=" << t.get_size() << ")";
        return dbg_builder->createBasicType(ss.str(), t.get_size() * 8, llvm::dwarf::DW_ATE_signed);
    }
    else if (t.is_float())
    {
        return dbg_builder->createBasicType("REAL(KIND=4)", 32, llvm::dwarf::DW_ATE_float);
    }
    else if (t.is_double())
    {
        return dbg_builder->createBasicType("REAL(KIND=8)", 64, llvm::dwarf::DW_ATE_float);
    }
    else if (t.is_char())
    {   
        // Should we honor platform dependent signedness?
        return dbg_builder->createBasicType("CHARACTER", 8, llvm::dwarf::DW_ATE_signed_char);
    }
    else if (t.is_void())
    {
        return dbg_builder->createUnspecifiedType("void");
    }
    else if (t.is_complex())
    {
        std::stringstream ss;
        ss << "COMPLEX(KIND=" << t.complex_get_base_type().get_size() << ")";
        std::string complex_typename = ss.str();

        return dbg_builder->createBasicType(complex_typename,
                t.get_size() * 8,
                llvm::dwarf::DW_ATE_complex_float);
    }
    else if ((t.is_fortran_array() || t.is_fortran_character())
            && !t.array_requires_descriptor())
    {
        // Statically sized arrays or VLAs
        // Unfortunately VLAs have to be represented in a very useless way
        // since DIBuilder does not seem to support DWARFv4 calculated
        // subranges yet.
        Nodecl::NodeclBase size = t.array_get_size();

        std::vector<llvm::Metadata*> subscripts;
        TL::Type current_type = t;
        while (current_type.is_fortran_array())
        {
            Nodecl::NodeclBase lower, upper;
            current_type.array_get_bounds(lower, upper);

            subscripts.push_back(dbg_builder->getOrCreateSubrange(
                lower.is_constant() ? const_value_cast_to_8(lower.get_constant()) : 0, 
                upper.is_constant() ? const_value_cast_to_8(upper.get_constant()) : 0));

            current_type = current_type.array_element();
        }

        llvm::DINodeArray array = dbg_builder->getOrCreateArray(subscripts);
        return dbg_builder->createArrayType(
                size.is_constant() ? const_value_cast_to_8(size.get_constant()) : 0,
                current_type.array_base_element().get_alignment_of(),
                get_debug_info_type(current_type.array_base_element()),
                array);
    }
    else if (t.is_fortran_array() && t.array_requires_descriptor())
    {
        // FIXME - LLVM is lacking in this area
        return dbg_builder->createUnspecifiedType("void");
    }
    else
    {
        internal_error("Cannot synthesize Debug Info type for type '%s'",
                       print_declarator(t.get_internal_type()));
    }
}

llvm::Value* FortranLLVM::eval_size_of_dimension(TL::Type t)
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

llvm::Value* FortranLLVM::eval_size_of_array(TL::Type t)
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

llvm::Value* FortranLLVM::array_descriptor_addr_dim_data(llvm::Value *descriptor_addr, int dimension, const std::string& field)
{
    llvm::Type *descriptor_type = descriptor_addr->getType()->getPointerElementType();
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


llvm::Value* FortranLLVM::eval_elements_of_dimension(Nodecl::NodeclBase expr, TL::Type t, llvm::Value *addr, int dimension)
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

llvm::Value* FortranLLVM::eval_elements_of_array(Nodecl::NodeclBase expr, TL::Type t, llvm::Value *addr)
{
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type", 0);
    llvm::Value *val_size = nullptr;
    int dimension = 0;
    while (t.is_fortran_array())
    {
        llvm::Value *current_size = eval_elements_of_dimension(expr, t, addr, dimension);

        if (val_size == nullptr)
            val_size = current_size;
        else
            val_size = ir_builder->CreateMul(val_size, current_size);

        t = t.array_element();
        dimension++;
    }

    return val_size;
}

void FortranLLVM::emit_variable(TL::Symbol sym)
{
    ERROR_CONDITION(!sym.is_variable(),
                    "Invalid symbol kind '%s'\n",
                    symbol_kind_name(sym.get_internal_symbol()));
    if (get_value(sym) != NULL)
        return;

    llvm::Value *array_size = nullptr;
    if (sym.get_type().is_fortran_array() && !sym.get_type().array_requires_descriptor())
    {
        // Emit size
        TrackLocation loc(this, sym.get_locus());

        TL::Type t = sym.get_type();
        array_size = eval_size_of_array(t);
    }

    llvm::Type *llvm_type = get_llvm_type(sym.get_type());
    llvm::Value *allocation = ir_builder->CreateAlloca(llvm_type, array_size, sym.get_name());
    map_symbol_to_value(sym, allocation);

    llvm::DINode::DIFlags flags = llvm::DINode::FlagZero;
    if (sym.is_saved_expression())
        flags |= llvm::DINode::FlagArtificial;
    llvm::DILocalVariable *dbg_var =
        dbg_builder->createAutoVariable(get_debug_scope(), sym.get_name(), dbg_info.file,
                sym.get_line(), get_debug_info_type(sym.get_type()),
                /* AlwaysPreserve */ false,
                flags);

    std::vector<int64_t> dbg_expr_ops;
    llvm::DIExpression *dbg_expr = dbg_builder->createExpression(dbg_expr_ops);

    dbg_builder->insertDeclare(allocation,
            dbg_var,
            dbg_expr,
            llvm::DILocation::get(llvm_context,
                sym.get_line(),
                sym.get_column(),
                get_debug_scope()),
            ir_builder->GetInsertBlock());

    // If this variable is ALLOCATABLE or a POINTER it must be set to zero
    if (sym.is_allocatable() || sym.get_type().is_pointer())
    {
        if ((sym.get_type().is_array()
             && sym.get_type().array_requires_descriptor())
            || (sym.get_type().is_pointer()
                && sym.get_type().points_to().is_array()))
        {
            FortranLLVM::TrackLocation loc(this, sym.get_locus());
            // This has created a descriptor. Set it to zero.
            // FIXME - I think there is a zeroinitializer for these cases.
            ir_builder->CreateStore(
                llvm::ConstantPointerNull::get(llvm_types.ptr_i8),
                array_descriptor_addr_base_addr(allocation));
        }
        else
        {
            internal_error("Not implemented yet", 0);
        }
    }
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

void FortranLLVM::fill_descriptor_info(int rank,
        TL::Type element_type,
        llvm::Value* descriptor_addr,
        llvm::Value* base_address,
        const std::vector<llvm::Value*> &lower_bounds,
        const std::vector<llvm::Value*> &upper_bounds,
        const std::vector<llvm::Value*> &strides)
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
    std::vector<llvm::Value*> descr_strides(rank, nullptr);
    for (int i = 0; i < rank; i++)
    {
        if (i == 0)
        {
            descr_strides[0] = strides[0];
            offset_value = ir_builder->CreateMul(
                    lower_bounds[0],
                    descr_strides[0]);
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
    offset_value = ir_builder->CreateSub(
            get_integer_value_64(0),
            offset_value);

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
        internal_error("Uknown type '%s' for array\n", print_declarator(element_type.get_internal_type()));

    int size_type = element_type.get_size();

    llvm::Value *dtype_value = get_integer_value_64(rank | (type_id << 3) | (size_type << 6));
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

void FortranLLVM::fill_descriptor_info(
    TL::Type array_type,
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

void FortranLLVM::gfortran_runtime_error(const locus_t* locus, const std::string &str)
{
    std::stringstream ss;
    ss << "At " << locus_to_str(locus);

    ir_builder->CreateCall(gfortran_rt.runtime_error_at.get(),
                           { ir_builder->CreateGlobalStringPtr(ss.str()),
                             ir_builder->CreateGlobalStringPtr("%s"),
                             ir_builder->CreateGlobalStringPtr(str) });
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
    auto add_type_int4 = [=](TL &t) { t.push_back(llvm_types.i32); };
    auto add_type_intio = [=](TL &t) { add_type_int4(t); };
    auto add_type_pint4 = [=](TL &t) { t.push_back(llvm_types.ptr_i32); };
    auto add_type_pchar = [=](TL &t) { t.push_back(llvm_types.ptr_i8); };
    auto add_type_char1 = [=](TL &t) { 
        t.push_back(llvm_types.i32); 
        add_type_pchar(t);
    };
    auto add_type_char2 = [=](TL &t) {
        add_type_pchar(t);
        t.push_back(llvm_types.i32);
    };
    auto add_type_pintio = [=](TL &t) { t.push_back(llvm_types.i64); };
    auto add_type_parray = [=](TL &t) { add_type_pchar(t); };
    auto add_type_pad = [=](TL &t) {
        const llvm::DataLayout &dl = current_module->getDataLayout();
        uint64_t size = 16 * dl.getPointerTypeSize(llvm_types.ptr_i8)
            + 32 * dl.getTypeAllocSize(llvm_types.i32);
        t.push_back(llvm::ArrayType::get(llvm_types.i8, size));
    };
    auto add_type_common = [=](TL &t) {
        t.push_back(gfortran_rt.st_parameter_common.get());
    };

#define IOPARM_START(name) \
    auto create_##name = [=]() { \
         llvm::StructType *ioparm_type_##name = llvm::StructType::create( \
             llvm_context, "st_parameter_" #name); \
         std::vector<llvm::Type *> name##_elements;
#define IOPARM(name, field_name, __, type) \
    add_type_##type(name##_elements); \
    fields[ioparm_type_##name].add_field(#field_name);
#define IOPARM_END(name) \
        ioparm_type_##name->setBody(name##_elements); \
        return static_cast<llvm::Type*>(ioparm_type_##name); };
    IOPARM_LIST_FIELDS
#undef IOPARM_END
#undef IOPARM
#undef IOPARM_START

    this->gfortran_rt.st_parameter_common = create_common;
    this->gfortran_rt.st_parameter_dt = create_dt;

    this->gfortran_rt.st_write = [&, this]() {
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { gfortran_rt.st_parameter_dt.get()->getPointerTo() },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_st_write",
                current_module.get());
    };

    this->gfortran_rt.transfer_character_write = [&, this]() {
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                    llvm_types.ptr_i8,
                    llvm_types.i32 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_transfer_character_write",
                current_module.get());
    };

    this->gfortran_rt.transfer_integer_write = [&, this]() {
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                    llvm_types.ptr_i8,
                    llvm_types.i32 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_transfer_integer_write",
                current_module.get());
    };

    this->gfortran_rt.transfer_real_write = [&, this]() { 
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                    llvm_types.ptr_i8,
                    llvm_types.i32 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_transfer_real_write",
                current_module.get()); 
    };

    this->gfortran_rt.transfer_complex_write = [&, this]() { 
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                    llvm_types.ptr_i8,
                    llvm_types.i32 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_transfer_complex_write",
                current_module.get()); 
    };

    this->gfortran_rt.transfer_logical_write = [&, this]() { 
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                    llvm_types.ptr_i8,
                    llvm_types.i32 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_transfer_logical_write",
                current_module.get()); 
    };

    this->gfortran_rt.transfer_array_write = [&, this]() { 
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                    llvm_types.ptr_i8,
                    llvm_types.i32,
                    llvm_types.i32 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_transfer_array_write",
                current_module.get()); 
    };

    this->gfortran_rt.st_write_done = [&, this]() { 
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { gfortran_rt.st_parameter_dt.get()->getPointerTo() },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_st_write_done",
                current_module.get());
    };

    this->gfortran_rt.set_args = [&, this]() {
        return llvm::Function::Create(
                llvm::FunctionType::get(
                    llvm_types.void_,
                    { llvm_types.i32,
                    llvm_types.ptr_i8->getPointerTo() },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_set_args",
                current_module.get());
    };

    this->gfortran_rt.set_options = [&, this]() {
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { llvm_types.i32,
                    llvm_types.ptr_i32 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_set_options",
                current_module.get());
    };

    this->gfortran_rt.stop_int = [&, this]() {
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { llvm_types.i32 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_stop_numeric_f08",
                current_module.get());
    };

    this->gfortran_rt.descriptor_dimension = [&, this]() -> llvm::Type *
    {
        // struct descriptor_dimension
        // {
        //   ptrdiff_t stride;
        //   ptrdiff_t lower_bound;
        //   ptrdiff_t upper_bound;
        // };
        // FIXME - We need the right ptrdiff_t here
        llvm::Type *pdiff = llvm_types.i64;
        llvm::StructType *t
            = llvm::StructType::create(llvm_context, "descriptor_dimension");
        fields[t].add_field("stride");
        fields[t].add_field("lower_bound");
        fields[t].add_field("upper_bound");
        t->setBody({pdiff, pdiff, pdiff});
        return t;
    };

    this->gfortran_rt.malloc = [&, this]()
    {
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.ptr_i8,
                    // FIXME - Make this a size_t
                    { llvm_types.i64 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "malloc",
                current_module.get());
    };

    this->gfortran_rt.free = [&, this]()
    {
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { llvm_types.ptr_i8 },
                    /* isVarArg */ false),
                llvm::GlobalValue::ExternalLinkage,
                "free",
                current_module.get());
    };

    this->gfortran_rt.runtime_error_at = [&, this]()
    {
        return llvm::Function::Create(
                llvm::FunctionType::get(llvm_types.void_,
                    { llvm_types.ptr_i8, llvm_types.ptr_i8 },
                    /* isVarArg */ true),
                llvm::GlobalValue::ExternalLinkage,
                "_gfortran_runtime_error_at",
                current_module.get());
    };
}

llvm::Type* FortranLLVM::get_gfortran_array_descriptor_type(TL::Type t)
{
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type", 0);

    int rank = t.fortran_rank();

    auto it = gfortran_rt.array_descriptor.find(rank);
    if (it != gfortran_rt.array_descriptor.end())
        return it->second;

    std::stringstream ss;
    ss << "descriptor_rank_" << rank;
    llvm::StructType *struct_type = llvm::StructType::create(llvm_context, ss.str());

    // template <int Rank>
    // struct descriptor
    // {
    //     void *base_addr;
    //     size_t offset;
    //     ptrdiff_t dtype;
    //     descriptor_dimension dim[Rank];
    // };

    std::vector<llvm::Type *> field_types;
    field_types.reserve(4);

    fields[struct_type].add_field("base_addr");
    field_types.push_back(llvm_types.ptr_i8);

    fields[struct_type].add_field("offset");
    // Fixme we need a sensible size_t here
    field_types.push_back(llvm_types.i64);

    fields[struct_type].add_field("dtype");
    // Fixme we need a sensible ptrdiff_t here
    field_types.push_back(llvm_types.i64);

    fields[struct_type].add_field("dim");
    field_types.push_back(
        llvm::ArrayType::get(gfortran_rt.descriptor_dimension.get(), rank));

    struct_type->setBody(field_types);

    gfortran_rt.array_descriptor.insert(std::make_pair(rank, struct_type));

    return struct_type;
}

void FortranLLVM::initialize_aliasing_info()
{
    // tbaa_root = md_builder->createTBAARoot("Fortran 95 aliasing");
}

void FortranLLVM::initialize_llvm_context()
{
    initialize_llvm_types();
    initialize_gfortran_runtime();
    initialize_aliasing_info();
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

    llvm::AttributeSet attributes;
    attributes = attributes.addAttribute(llvm_context,
            llvm::AttributeSet::FunctionIndex,
            llvm::Attribute::NoRecurse);
    attributes = attributes.addAttribute(llvm_context,
            llvm::AttributeSet::FunctionIndex,
            llvm::Attribute::UWTable);
    attributes = attributes.addAttribute(llvm_context,
            llvm::AttributeSet::FunctionIndex,
            llvm::Attribute::NoUnwind);
    llvm::Constant *c = current_module->getOrInsertFunction(
            "main",
            llvm::FunctionType::get(
                llvm_types.i32,
                { llvm_types.i32,
                llvm_types.ptr_i8->getPointerTo() },
                /* isVarArg */ false),
            attributes);

    llvm::Function *fun = llvm::cast<llvm::Function>(c);

    set_current_function(fun);
    llvm::BasicBlock *entry_basic_block = llvm::BasicBlock::Create(llvm_context, "entry", fun);
    set_current_block(entry_basic_block);

    std::vector<llvm::Value *> main_args;
    for (llvm::Argument &v : fun->args())
    {
        main_args.push_back(&v);
    }
    ir_builder->CreateCall(gfortran_rt.set_args.get(), main_args);

    ir_builder->CreateCall(
        gfortran_rt.set_options.get(),
        { get_integer_value_32(9),
          ir_builder->CreatePointerCast(options, llvm_types.ptr_i32) });

    ir_builder->CreateCall(fortran_program, {});

    ir_builder->CreateRet(get_integer_value_32(0));

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
