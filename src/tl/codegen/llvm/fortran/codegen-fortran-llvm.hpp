/*--------------------------------------------------------------------
  (C) Copyright 2006-2015 Barcelona Supercomputing Center
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

#ifndef CODEGEN_FORTRAN_LLVM_HPP
#define CODEGEN_FORTRAN_LLVM_HPP

#include "codegen-phase.hpp"

#include "codegen-fortran.hpp"
#include <llvm/IR/Module.h>

namespace Codegen
{
    class FortranLLVM : public CodegenPhase
    {
        public:
            FortranLLVM();

            virtual void codegen(const Nodecl::NodeclBase&, std::ostream* out);
            virtual void codegen_cleanup();

            void visit(const Nodecl::TopLevel& node);
            void visit(const Nodecl::FunctionCode& node);
            // void visit(const Nodecl::Context& node);
            // void visit(const Nodecl::CompoundStatement& node);
            // void visit(const Nodecl::ExpressionStatement& node);
            // void visit(const Nodecl::ObjectInit& node);
            // void visit(const Nodecl::Plus& node);
            // void visit(const Nodecl::Neg& node);
            // void visit(const Nodecl::LogicalNot& node);
            // void visit(const Nodecl::Mul& node);
            // void visit(const Nodecl::Div& node);
            // void visit(const Nodecl::Mod& node);
            // void visit(const Nodecl::Add& node);
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
            // void visit(const Nodecl::IntegerLiteral& node);
            // void visit(const Nodecl::ComplexLiteral& node);
            // void visit(const Nodecl::FloatingLiteral& node);
            // void visit(const Nodecl::Symbol& node);
            // void visit(const Nodecl::Assignment& node);
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

            // void visit(const Nodecl::ErrExpr& node);
            // void visit(const Nodecl::ErrStatement& node);

        private:
            std::unique_ptr<llvm::Module> current_module;
            llvm::LLVMContext llvm_context;

            Codegen::FortranBase base;
    };
}

#endif // CODEGEN_FORTRAN_LLVM_HPP
