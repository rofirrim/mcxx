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
#include "llvm/IR/DIBuilder.h"

#include <map>

namespace Codegen
{
    template <typename Object>
    struct LazyObjectPtr
    {
        private:
            Object *obj = nullptr;
            std::function<Object*()> create;
        public:
            template <typename Functor>
            LazyObjectPtr& operator=(Functor f)
            {
                ERROR_CONDITION(obj != nullptr,
                    "Object has already been constructed", 0);
                create = f;
                return *this;
            }

            Object* get()
            {
                if (obj == nullptr)
                    obj = create();
                return obj;
            }
    };

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
        void visit(const Nodecl::EmptyStatement &node);
        void visit(const Nodecl::FortranStopStatement &node);
        void visit(const Nodecl::ObjectInit &node);
        void visit(const Nodecl::IfElseStatement& node);
        void visit(const Nodecl::ForStatement& node);
        void visit(const Nodecl::FortranPrintStatement& node);
        void visit(const Nodecl::FortranAllocateStatement& node);
        void visit(const Nodecl::FortranDeallocateStatement& node);
        void visit(const Nodecl::WhileStatement& node);
        void visit(const Nodecl::SwitchStatement &node);
        void visit(const Nodecl::CaseStatement &node);
        void visit(const Nodecl::DefaultStatement &node);

      private:
        Codegen::FortranBase base;

        llvm::LLVMContext llvm_context;
        std::unique_ptr<llvm::Module> current_module;
        std::unique_ptr<llvm::IRBuilder<> > ir_builder;
        std::unique_ptr<llvm::DIBuilder > dbg_builder;

        struct FunctionInfo
        {
            llvm::Function *function;
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
            LazyObjectPtr<llvm::Type> st_parameter_common;
            LazyObjectPtr<llvm::Type> st_parameter_dt;

            LazyObjectPtr<llvm::Function> st_write;
            LazyObjectPtr<llvm::Function> transfer_character_write;
            LazyObjectPtr<llvm::Function> transfer_integer_write;
            LazyObjectPtr<llvm::Function> transfer_real_write;
            LazyObjectPtr<llvm::Function> transfer_complex_write;
            LazyObjectPtr<llvm::Function> transfer_logical_write;
            LazyObjectPtr<llvm::Function> transfer_array_write;
            LazyObjectPtr<llvm::Function> st_write_done;

            LazyObjectPtr<llvm::Function> set_args;
            LazyObjectPtr<llvm::Function> set_options;

            LazyObjectPtr<llvm::Function> stop_int;

            LazyObjectPtr<llvm::Type> descriptor_dimension;
            std::map<int, llvm::Type*> array_descriptor;

            LazyObjectPtr<llvm::Function> malloc;
            LazyObjectPtr<llvm::Function> free;
            LazyObjectPtr<llvm::Function> runtime_error_at;
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
            llvm::PointerType* ptr_i8;
            llvm::PointerType* ptr_i32;
            llvm::PointerType* ptr_i64;
        } llvm_types;

        std::map<llvm::Type*, FieldMap> fields;

        struct DebugInfo
        {
            llvm::DIFile* file = nullptr;
            llvm::DISubprogram* function = nullptr;
            std::vector<llvm::DIScope *> stack_debug_scope;

            std::map<std::string, llvm::DIModule*> module_map;

            DebugInfo() = default;
            DebugInfo& operator=(const DebugInfo&) = default;

            void reset() {
                *this = DebugInfo();
            }
        } dbg_info;

        struct SwitchInfo
        {
            llvm::BasicBlock *end_block = nullptr;

            llvm::Value *value = nullptr;
            Nodecl::NodeclBase expr;

            Nodecl::NodeclBase default_case;
        };

        std::vector<SwitchInfo> switch_info_stack;

        void push_switch()
        {
            switch_info_stack.push_back(SwitchInfo());
        }

        SwitchInfo& current_switch()
        {
            ERROR_CONDITION(switch_info_stack.empty(), "Invalid empty switch stack", 0);
            return switch_info_stack.back();
        }

        void pop_switch()
        {
            ERROR_CONDITION(switch_info_stack.empty(), "Invalid empty switch stack", 0);
            switch_info_stack.pop_back();
        }

      private:
        void initialize_llvm_context();
        void initialize_llvm_types();
        void initialize_gfortran_runtime();
        llvm::Type* get_gfortran_array_descriptor_type(TL::Type t);
        void gfortran_runtime_error(const locus_t *, const std::string &str);

        void fill_descriptor_info(
            int rank,
            TL::Type element_type,
            llvm::Value *descriptor_addr,
            llvm::Value *base_address,
            const std::vector<llvm::Value *> &lower_bounds,
            const std::vector<llvm::Value *> &upper_bounds,
            const std::vector<llvm::Value *> &strides);

        void fill_descriptor_info(
            TL::Type array_type,
            llvm::Value *descriptor_addr,
            llvm::Value *base_address,
            const TL::ObjectList<Nodecl::NodeclBase> &array_sizes);

        void fill_descriptor_info(
                TL::Type array_type,
                llvm::Value *descriptor_addr,
                llvm::Value *base_address);

        // Debug info
        void push_debug_scope(llvm::DIScope *dbg_scope)
        {
            dbg_info.stack_debug_scope.push_back(dbg_scope);
        }
        void pop_debug_scope()
        {
            ERROR_CONDITION(dbg_info.stack_debug_scope.empty(), "Stack of debug scopes is empty", 0);
            dbg_info.stack_debug_scope.pop_back();
        }
        llvm::DIScope *get_debug_scope()
        {
            ERROR_CONDITION(dbg_info.stack_debug_scope.empty(), "Stack of debug scopes is empty", 0);
            return dbg_info.stack_debug_scope.back();
        }

        llvm::Type *get_llvm_type(TL::Type t);
        llvm::DIType *get_debug_info_type(TL::Type t);

        llvm::DIModule *get_module(const std::string& name)
        {
            decltype(dbg_info.module_map)::iterator it = dbg_info.module_map.find(name);
            if (it != dbg_info.module_map.end())
                return it->second;

            llvm::DIModule *module = dbg_builder->createModule(dbg_info.file,
                    name,
                    /* ConfigurationMacros */ llvm::StringRef(),
                    /* IncludePath */ llvm::StringRef(), // TODO: paths specified in -J ?
                    /* ISysRoot */ llvm::StringRef());
            dbg_info.module_map.insert(std::make_pair(name, module));
            return module;
        }

        struct TrackLocation
        {
            private:
                FortranLLVM *visitor;
                llvm::DebugLoc old_debug_loc;
            public:
                TrackLocation(FortranLLVM* v, const locus_t* locus)
                    : visitor(v), old_debug_loc(visitor->ir_builder->getCurrentDebugLocation())
                {
                    ERROR_CONDITION(!llvm::isa<llvm::DILocalScope>(v->get_debug_scope()), "Invalid scope for location", 0);
                    visitor->ir_builder->SetCurrentDebugLocation(
                            llvm::DILocation::get(visitor->llvm_context,
                                locus_get_line(locus),
                                locus_get_column(locus),
                                v->get_debug_scope()));
                }

                TrackLocation(FortranLLVM* v, Nodecl::NodeclBase n)
                    : TrackLocation(v, n.get_locus())
                {
                }

                ~TrackLocation()
                {
                    visitor->ir_builder->SetCurrentDebugLocation(old_debug_loc);
                }
        };

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
            ir_builder->SetInsertPoint(bb);
        }
        llvm::BasicBlock* get_current_block()
        {
            return ir_builder->GetInsertBlock();
        }

        void emit_variable(TL::Symbol sym);

        // Takes into account region arrays
        llvm::Value* eval_elements_of_dimension(TL::Type t);
        llvm::Value* eval_elements_of_array(TL::Type sym);

        // Only considers array types
        llvm::Value* eval_size_of_dimension(TL::Type t);
        llvm::Value* eval_size_of_array(TL::Type sym);
        llvm::Value* eval_length_of_character(TL::Type sym);

        llvm::Value *gep_for_field(
            llvm::Type *struct_type,
            llvm::Value *addr,
            const std::vector<std::string> &fields);

        llvm::Value *constant_string(std::string &str);

        // Compute the number of bytes used by an expression ignoring references
        llvm::Value *eval_sizeof(Nodecl::NodeclBase n);
        llvm::Value *eval_sizeof_64(Nodecl::NodeclBase n);
        llvm::Value *eval_sizeof(TL::Type);
        llvm::Value *eval_sizeof_64(TL::Type);

        llvm::Constant* get_integer_value_N(int64_t v, llvm::Type* t, int bits);

        llvm::Constant* get_integer_value(int64_t v, TL::Type t);

        llvm::Constant* get_integer_value_32(int64_t v);
        llvm::Constant* get_integer_value_64(int64_t v);

        void emit_main(llvm::Function *fortran_program);

        friend class FortranVisitorLLVMEmitVariables;
        friend class FortranVisitorLLVMExpression;
    };
}

#endif // CODEGEN_FORTRAN_LLVM_HPP
