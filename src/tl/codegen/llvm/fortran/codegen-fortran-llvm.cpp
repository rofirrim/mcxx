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

#include "tl-compilerphase.hpp"
#include "codegen-fortran-llvm.hpp"
#include "fortran03-buildscope.h"
#include "fortran03-scope.h"
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

#include <llvm/Support/raw_os_ostream.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/Argument.h>
#include <llvm/IR/Function.h>

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

void FortranLLVM::visit(const Nodecl::TopLevel &node)
{
    std::unique_ptr<llvm::Module> old_module;
    std::swap(old_module, current_module);

    current_module = llvm::make_unique<llvm::Module>(
        TL::CompilationProcess::get_current_file().get_filename(),
        llvm_context);

    current_module->setTargetTriple(llvm::sys::getDefaultTargetTriple());

    ir_builder = std::unique_ptr<llvm::IRBuilder<> >(
        new llvm::IRBuilder<>(llvm_context));

    walk(node.get_top_level());

    llvm::raw_os_ostream ros(*file);
    current_module->print(ros,
                          /* AssemblyAnnotationWriter */ nullptr);
    current_module->dump();

    std::swap(old_module, current_module);
}

void FortranLLVM::visit(const Nodecl::FunctionCode &node)
{
    // Create a function with the proper type
    TL::Symbol sym = node.get_symbol();

    llvm::Type *function_type = get_llvm_type(sym.get_type());

    llvm::Constant *c = current_module->getOrInsertFunction(
        sym.get_name() /* FIXME - Mangled name for modules members! */,
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

    set_function(fun);
    set_current_block(llvm::BasicBlock::Create(llvm_context, "entry", fun));
    walk(node.get_statements());
    // FIXME: Finish function. Use proper value!
    ir_builder->CreateRet(nullptr);
    clear_function();
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
    walk(node.get_nest());
}

void FortranLLVM::visit(const Nodecl::Assignment &node)
{
    llvm::Value *vlhs = visit_expression(node.get_lhs());
    llvm::Value *vrhs = visit_expression(node.get_rhs());

    ir_builder->CreateStore(vrhs, vlhs);
}

class FortranVisitorLLVMExpression : public Nodecl::NodeclVisitor<void>
{
  private:
    FortranLLVM *llvm_visitor;
    llvm::Value *value = nullptr;

  public:
    FortranVisitorLLVMExpression(FortranLLVM *llvm_visitor)
        : llvm_visitor(llvm_visitor)
    {
    }

    llvm::Value *get_value()
    {
        return value;
    }

    void unhandled_node(const Nodecl::NodeclBase &n)
    {
        internal_error("Unexpected node '%s'\n",
                       ast_print_node_type(n.get_kind()))
    }

    void visit(const Nodecl::Symbol &node)
    {
        value = llvm_visitor->get_value(node.get_symbol());
        ERROR_CONDITION(value == nullptr,
                        "No llvm::Value mapping for symbol '%s'\n",
                        node.get_symbol().get_name().c_str());
    }

    void visit(const Nodecl::Conversion& node)
    {
        // A conversion coalesces more than one conversion including a "load"
        // For now just check that this is a pure load
        llvm::Value *vnest = llvm_visitor->visit_expression(node.get_nest());
        bool is_load = 
            node.get_type().is_same_type(node.get_nest().get_type().no_ref());
        ERROR_CONDITION(!is_load, "Not yet implemented", 0);

        value = llvm_visitor->ir_builder->CreateLoad(vnest);
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
            llvm::Value *vlhs = llvm_visitor->visit_expression(node.get_lhs());
            llvm::Value *vrhs = llvm_visitor->visit_expression(node.get_rhs());

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
    // void visit(const Nodecl::GreaterThan& node);
    // void visit(const Nodecl::GreaterOrEqualThan& node);
    // void visit(const Nodecl::LogicalAnd& node);
    // void visit(const Nodecl::LogicalOr& node);
    // void visit(const Nodecl::Power& node);
    // void visit(const Nodecl::Concat& node);
    // void visit(const Nodecl::ClassMemberAccess& node);
    // void visit(const Nodecl::Range& node);
    // void visit(const Nodecl::StringLiteral& node);
    // void visit(const Nodecl::Text& node);
    // void visit(const Nodecl::StructuredValue& node);
    // void visit(const Nodecl::BooleanLiteral& node);
    void visit(const Nodecl::IntegerLiteral& node)
    {
        // FIXME: INTEGER(16) ?
        value = llvm::Constant::getIntegerValue(
                llvm_visitor->get_llvm_type(node.get_type()),
                llvm::APInt(node.get_type().get_size() * 8, // FIXME
                    const_value_cast_to_8(node.get_constant()),
                    /* is_signed */ true));
    }
    // void visit(const Nodecl::ComplexLiteral& node);
    // void visit(const Nodecl::FloatingLiteral& node);
    // void visit(const Nodecl::Symbol& node);
    // void visit(const Nodecl::Equal& node);
    // void visit(const Nodecl::Different& node);
    // void visit(const Nodecl::Dereference& node);
    // void visit(const Nodecl::Reference& node);
    // void visit(const Nodecl::ParenthesizedExpression& node);
    // void visit(const Nodecl::ArraySubscript& node);
    // void visit(const Nodecl::FunctionCall& node);
    // void visit(const Nodecl::FortranActualArgument& node);
    // void visit(const Nodecl::EmptyStatement& node);
    // void visit(const Nodecl::IfElseStatement& node);
    // void visit(const Nodecl::ReturnStatement& node);
    // void visit(const Nodecl::LabeledStatement& node);
    // void visit(const Nodecl::GotoStatement& node);
    // void visit(const Nodecl::ForStatement& node);
    // void visit(const Nodecl::WhileStatement& node);
    // void visit(const Nodecl::RangeLoopControl& node);
    // void visit(const Nodecl::SwitchStatement& node);
    // void visit(const Nodecl::CaseStatement& node);
    // void visit(const Nodecl::DefaultStatement& node);
    // void visit(const Nodecl::BreakStatement& node);
    // void visit(const Nodecl::ContinueStatement& node);
    // void visit(const Nodecl::FortranIoSpec& node);
    // void visit(const Nodecl::FortranPrintStatement& node);
    // void visit(const Nodecl::FortranWriteStatement& node);
    // void visit(const Nodecl::FortranReadStatement& node);
    // void visit(const Nodecl::FortranStopStatement& node);
    // void visit(const Nodecl::FortranPauseStatement& node);
    // void visit(const Nodecl::FortranComputedGotoStatement& node);
    // void visit(const Nodecl::FortranIoStatement& node);
    // void visit(const Nodecl::FortranOpenStatement& node);
    // void visit(const Nodecl::FortranCloseStatement& node);
    // void visit(const Nodecl::FortranAllocateStatement& node);
    // void visit(const Nodecl::FortranDeallocateStatement& node);
    // void visit(const Nodecl::FortranNullifyStatement& node);
    // void visit(const Nodecl::FortranArithmeticIfStatement& node);
    // void visit(const Nodecl::FortranLabelAssignStatement& node);
    // void visit(const Nodecl::FortranAssignedGotoStatement& node);
    // void visit(const Nodecl::FortranEntryStatement& node);
    // void visit(const Nodecl::FortranImpliedDo& node);
    // void visit(const Nodecl::FortranData& node);
    // void visit(const Nodecl::FortranEquivalence& node);
    // void visit(const Nodecl::FortranAlternateReturnArgument& node);
    // void visit(const Nodecl::FortranAlternateReturnStatement& node);
    // void visit(const Nodecl::FortranForall& node);
    // void visit(const Nodecl::FortranWhere& node);
    // void visit(const Nodecl::FortranBozLiteral& node);
    // void visit(const Nodecl::FortranHollerith& node);
    // void visit(const Nodecl::FortranUse& node);
    // void visit(const Nodecl::FortranUseOnly& node);
    // void visit(const Nodecl::FieldDesignator& node);
    // void visit(const Nodecl::IndexDesignator& node);
    // void visit(const Nodecl::Conversion& node);
    // void visit(const Nodecl::UnknownPragma& node);
    // void visit(const Nodecl::PragmaCustomDeclaration& node);
    // void visit(const Nodecl::PragmaCustomClause& node);
    // void visit(const Nodecl::PragmaCustomLine& node);
    // void visit(const Nodecl::PragmaCustomStatement& node);
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

llvm::Value *FortranLLVM::visit_expression(Nodecl::NodeclBase n)
{
    FortranVisitorLLVMExpression v(this);
    v.walk(n);
    return v.get_value();
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
    else if (t.is_signed_integral())
    {
        return llvm::IntegerType::get(llvm_context, t.get_size() * 8);
    }
    else if (t.is_void())
    {
        return llvm::Type::getVoidTy(llvm_context);
    }
    else
    {
        internal_error("Cannot convert type '%s' to LLVM type",
                       print_declarator(t.get_internal_type()));
    }
}

}


EXPORT_PHASE(Codegen::FortranLLVM)
