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
#include "fortran03-mangling.h"

namespace Codegen
{

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
        if (!sym.is_from_module() && sym.is_variable()
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
    FortranVisitorLLVMEmitVariables(FortranLLVM *llvm_visitor,
                                    TL::Scope current_scope)
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
    llvm::DISubroutineType *dbg_type = llvm::cast<llvm::DISubroutineType>(
        get_debug_info_type(function_type));
    llvm::DISubprogram *dbg_subprogram
        = dbg_builder->createFunction(get_debug_scope(),
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
            // We allow OPTIONAL be NULL (I think implementing OPTIONAL this way
            // violates the standard)
            llvm::AttributeSet attrs = fun->getAttributes();
            attrs = attrs.addAttribute(
                llvm_context, dbg_argno, llvm::Attribute::NonNull);
            if (!s.get_type().no_ref().is_fortran_array())
                attrs = attrs.addAttribute(
                    llvm_context,
                    { (unsigned)dbg_argno },
                    llvm::Attribute::get(llvm_context,
                                         llvm::Attribute::Dereferenceable,
                                         s.get_type().no_ref().get_size()));
            fun->setAttributes(attrs);
        }

        v->setName(s.get_name());
        map_symbol_to_value(s, v);

        std::vector<int64_t> dbg_expr_ops;
        llvm::DIExpression *dbg_expr
            = dbg_builder->createExpression(dbg_expr_ops);

        llvm::DILocalVariable *dbg_param = dbg_builder->createParameterVariable(
            get_debug_scope(),
            s.get_name(),
            dbg_argno,
            dbg_info.file,
            s.get_line(),
            get_debug_info_type(s.get_type()));
        dbg_builder->insertDeclare(
            v,
            dbg_param,
            dbg_expr,
            llvm::DILocation::get(
                llvm_context, s.get_line(), s.get_column(), get_debug_scope()),
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
    FortranVisitorLLVMEmitVariables e(
        this, nodecl_get_decl_context(context.get_internal_nodecl()));
    e.emit_variables(context.get_in_context());

    // Emit statements
    walk(context.get_in_context());

    if (sym.is_fortran_main_program() || sym.get_type().returns().is_void())
    {
        ir_builder->CreateRet(nullptr);
    }
    else
    {
        // Return the value in the return variable
        TL::Symbol return_variable = sym.get_result_variable();
        ERROR_CONDITION(
            !return_variable.is_valid(), "Result variable is missing?", 0);
        // Make sure it has been emitted in case it has not been referenced at
        // all
        emit_variable(return_variable);
        llvm::Value *addr_ret_val = get_value(return_variable);
        llvm::Value *value_ret_val = ir_builder->CreateLoad(addr_ret_val);
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
    // a FunctionCode or if using BLOCK .. END BLOCK. For the FunctionCode case,
    // we handle it
    // in the visitor of FunctionCode so this code will never be run by that
    // case.
    llvm::DILexicalBlock *lexical_block = dbg_builder->createLexicalBlock(
        get_debug_scope(), dbg_info.file, node.get_line(), node.get_column());
    push_debug_scope(lexical_block);

    // Emit variables (if any)
    FortranVisitorLLVMEmitVariables e(
        this, nodecl_get_decl_context(node.get_internal_nodecl()));
    e.emit_variables(node.get_in_context());

    walk(node.get_in_context());
    pop_debug_scope();
}
}
