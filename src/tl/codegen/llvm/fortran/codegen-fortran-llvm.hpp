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
#include "llvm/IR/Module.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/IRBuilder.h"

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
        void visit(const Nodecl::IfElseStatement& node);
        void visit(const Nodecl::ForStatement& node);
        void visit(const Nodecl::FortranPrintStatement& node);

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

        struct FieldMap
        {
            private:
                typedef std::map<std::string, int> field_map_t;
                field_map_t field_map;
                int idx = 0;
            public:
                int operator[](const std::string& name)
                {
                    field_map_t::iterator it = field_map.find(name);
                    ERROR_CONDITION(it == field_map.end(), "Invalid field name '%s'\n", name.c_str());

                    return it->second;
                }

                void add_field(const std::string &str)
                {
                    field_map[str] = idx;
                    idx++;
                }
        };

        struct GfortranRuntime
        {
            // Input/Output
            llvm::Type* st_parameter_common;
            llvm::Type* st_parameter_dt;

            llvm::Function *st_write;
            llvm::Function *transfer_character_write;
            llvm::Function *transfer_integer_write;
            llvm::Function *transfer_real_write;
            llvm::Function *st_write_done;

            llvm::Function *set_args;
            llvm::Function *set_options;
        } gfortran_rt;

        struct LLVMTypes
        {
            llvm::Type* i1;
            llvm::Type* i8;
            llvm::Type* i16;
            llvm::Type* i32;
            llvm::Type* i64;

            llvm::Type* f32;
            llvm::Type* f64;

            llvm::Type* void_;
            llvm::Type* ptr_i8;
            llvm::Type* ptr_i32;
            llvm::Type* ptr_i64;
        } llvm_types;

        std::map<llvm::Type*, FieldMap> fields;

      private:
        void initialize_llvm_context();
        void initialize_llvm_types();
        void initialize_gfortran_runtime();

        llvm::Type *get_llvm_type(TL::Type t);

        // Evaluates a Fortran expression
        llvm::Value *eval_expression(Nodecl::NodeclBase n);

        // Evaluates a Fortran expression but if it was not a lvalue reference
        // then it creates a temporary for it. Uses make_temporary
        llvm::Value *eval_expression_to_memory(Nodecl::NodeclBase n);

        // Promotes an expression known to just yield a value to a temporary
        llvm::Value *make_temporary(llvm::Value *v);

        // Map symbols to llvm::Value*
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

        // Current function
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

        // Basic block
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

        // Basic block where we emit allocas
        std::vector<llvm::BasicBlock*> stack_alloca_bb;
        void push_allocating_block(llvm::BasicBlock *b)
        {
            stack_alloca_bb.emplace_back(b);
        }
        void pop_allocating_block()
        {
            ERROR_CONDITION(stack_alloca_bb.empty(), "Stack of allocating basic blocks is empty", 0);
            stack_alloca_bb.pop_back();
        }
        llvm::BasicBlock* allocating_block()
        {
            ERROR_CONDITION(stack_alloca_bb.empty(), "Stack of allocating basic blocks is empty", 0);
            return stack_alloca_bb.back();
        }
        // Intentionally similar to CreateAlloca but creates it in the allocating_block
        llvm::Value *create_alloca(llvm::Type *t,
                                   llvm::Value *array_size = nullptr,
                                   const llvm::Twine &name = "");

        void emit_variable(TL::Symbol sym);

        llvm::Value *gep_for_field(
            llvm::Type *struct_type,
            llvm::Value *addr,
            const std::vector<std::string> &fields);

        llvm::Value *constant_string(std::string &str);

        // Compute the number of bytes used by an expression ignoring references
        llvm::Value *eval_sizeof(Nodecl::NodeclBase n);

        llvm::Constant* get_integer_value_N(int64_t v, llvm::Type* t, int bits);

        llvm::Constant* get_integer_value(int64_t v, TL::Type t);

        llvm::Constant* get_integer_value_32(int64_t v);
        llvm::Constant* get_integer_value_64(int64_t v);

        void emit_main(llvm::Function *fortran_program);

        friend class FortranVisitorLLVMExpression;
    };
}

#endif // CODEGEN_FORTRAN_LLVM_HPP
