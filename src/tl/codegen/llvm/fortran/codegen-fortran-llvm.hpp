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
#include <llvm/IR/Module.h>
#include <llvm/IR/IRBuilder.h>

#include <map>

namespace Codegen
{
    class FortranLLVM : public CodegenPhase
    {
      public:
        FortranLLVM();

        virtual void codegen(const Nodecl::NodeclBase &, std::ostream *out);
        virtual void codegen_cleanup();

        virtual const char* get_extension() {
            // We want the output files be .ll
            return ".ll";
        }

        void visit(const Nodecl::TopLevel &node);
        void visit(const Nodecl::FunctionCode &node);
        void visit(const Nodecl::Context &node);
        void visit(const Nodecl::CompoundStatement &node);
        void visit(const Nodecl::ExpressionStatement &node);
        void visit(const Nodecl::Assignment &node);
        void visit(const Nodecl::IfElseStatement& node);
        void visit(const Nodecl::ForStatement& node);

      private:
        Codegen::FortranBase base;

        llvm::LLVMContext llvm_context;
        std::unique_ptr<llvm::Module> current_module;
        std::unique_ptr<llvm::IRBuilder<> > ir_builder;

        struct FunctionInfo
        {
            llvm::Function *function;
            llvm::BasicBlock *current_block;
            std::map<TL::Symbol, llvm::Value *> mapping;
        } function_info;

      private:
        llvm::Type *get_llvm_type(TL::Type t);

        llvm::Value *visit_expression(Nodecl::NodeclBase n);

        void clear_mappings()
        {
            function_info.mapping.clear();
        }
        void map_symbol_to_value(TL::Symbol s, llvm::Value *v)
        {
            function_info.mapping[s] = v;
        }
        llvm::Value *get_value(TL::Symbol s)
        {
            return function_info.mapping[s];
        }

        llvm::Function *get_current_function() 
        {
            return function_info.function;
        }
        void set_current_function(llvm::Function *fun)
        {
            ERROR_CONDITION(fun == NULL, "Invalid function", 0);
            function_info.function = fun;
        }
        void clear_current_function()
        {
            function_info.function = nullptr;
        }
        void set_current_block(llvm::BasicBlock *bb)
        {
            ERROR_CONDITION(bb == NULL, "Invalid block", 0);
            function_info.current_block = bb;
            ir_builder->SetInsertPoint(bb);
        }
        llvm::BasicBlock* get_current_block()
        {
            return function_info.current_block;
        }
        void emit_variable(TL::Symbol sym);

        friend class FortranVisitorLLVMExpression;
    };
}

#endif // CODEGEN_FORTRAN_LLVM_HPP
