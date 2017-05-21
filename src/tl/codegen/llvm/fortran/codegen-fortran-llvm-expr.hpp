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


#ifndef CODEGEN_FORTRAN_LLVM_EXPR_HPP
#define CODEGEN_FORTRAN_LLVM_EXPR_HPP

#include "codegen-fortran-llvm.hpp"

namespace Codegen
{

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

    void visit(const Nodecl::Symbol &node);

    llvm::Value *scalar_conversion(TL::Type dest,
                                   TL::Type orig,
                                   llvm::Value *value_nest);

    void visit(const Nodecl::Conversion &node);

    bool is_scalar_to_array(const Nodecl::NodeclBase &n);

    struct LoopInfoOp
    {
        std::vector<llvm::Value *> idx_var;
        std::vector<llvm::BasicBlock *> block_check;
        std::vector<llvm::BasicBlock *> block_body;
        std::vector<llvm::BasicBlock *> block_end;
    };

    void create_loop_header_for_array_op(Nodecl::NodeclBase expr,
                                         TL::Type t,
                                         llvm::Value *addr,
                                         /* out */
                                         LoopInfoOp &loop_info_op);

    void create_loop_footer_for_array_op(TL::Type t,
                                         const LoopInfoOp &loop_info_op);

    llvm::Value *address_array_ith_element_via_descriptor(
        const Nodecl::NodeclBase &array,
        TL::Type t,
        llvm::Value *descr_address,
        const std::vector<llvm::Value *> indexes);

    llvm::Value *address_array_ith_element_via_pointer_arithmetic(
        TL::Type t,
        llvm::Value *base_address,
        const std::vector<llvm::Value *> indexes);

    llvm::Value *address_array_ith_element(
        const Nodecl::NodeclBase &array,
        TL::Type t,
        llvm::Value *base_address,
        const std::vector<llvm::Value *> indexes);

    std::vector<llvm::Value *> derref_indexes(
        const std::vector<llvm::Value *> v);

    template <typename Creator>
    void array_assignment(const Nodecl::NodeclBase &lhs,
                          const Nodecl::NodeclBase &rhs,
                          TL::Type lhs_type,
                          TL::Type rhs_type,
                          llvm::Value *lhs_addr,
                          llvm::Value *rhs_addr,
                          Creator create_store);

    typedef std::function<void(llvm::Value *addr, llvm::Value *value)> AssigOp;
    AssigOp get_assig_op(TL::Type lhs_type, TL::Type rhs_type);

    void visit(const Nodecl::ParenthesizedExpression &node);

    void visit(const Nodecl::Assignment &node);

    void visit(const Nodecl::BooleanLiteral &node);
    void visit(const Nodecl::ComplexLiteral &node);
    void visit(const Nodecl::StringLiteral &node);
    void visit(const Nodecl::IntegerLiteral &node);
    void visit(const Nodecl::FloatingLiteral &node);

    void visit(const Nodecl::ObjectInit &node);

    void visit(const Nodecl::Neg &node);
    void visit(const Nodecl::Plus &node);

    void visit(const Nodecl::LogicalAnd &node);
    void visit(const Nodecl::LogicalOr &node);
    void visit(const Nodecl::LogicalNot &node);

    template <typename Create>
    llvm::Value *binary_operator_scalar(Nodecl::NodeclBase lhs,
                                        Nodecl::NodeclBase rhs,
                                        llvm::Value *lhs_val,
                                        llvm::Value *rhs_val,
                                        Create create);

    llvm::Value *compute_offset_from_linear_element(TL::Type t,
                                                    llvm::Value *idx_value);

    template <typename Create>
    void binary_operator_array(const Nodecl::NodeclBase &node,
                               const Nodecl::NodeclBase &lhs,
                               const Nodecl::NodeclBase &rhs,
                               Create create);

    template <typename Node, typename Create>
    void binary_operator(const Node node, Create create);

    template <typename NonStrictOpt>
    void binary_operator_array_non_strict(const Nodecl::NodeclBase &node,
                                          const Nodecl::NodeclBase &lhs,
                                          const Nodecl::NodeclBase &rhs,
                                          NonStrictOpt non_strict_op);

    template <typename Node, typename NonStrictOp>
    void binary_operator_non_strict(const Node node, NonStrictOp non_strict_op);

    template <typename Create>
    llvm::Value *unary_operator_scalar(Nodecl::NodeclBase rhs,
                                       llvm::Value *rhs_val,
                                       Create create);

    template <typename Create>
    void unary_operator_array(const Nodecl::NodeclBase &node,
                              const Nodecl::NodeclBase &rhs,
                              Create create);

    template <typename Node, typename Create>
    void unary_operator(const Node node, Create create);

    llvm::Value *create_complex_value(TL::Type complex_type,
                                      llvm::Value *real,
                                      llvm::Value *imag);

    typedef std::function<llvm::Value *(
        Nodecl::NodeclBase, Nodecl::NodeclBase, llvm::Value *, llvm::Value *)>
        BinaryOpCreator;
    BinaryOpCreator choose_arithmetic_creator(TL::Type t,
                                              BinaryOpCreator create_integer,
                                              BinaryOpCreator create_real,
                                              BinaryOpCreator create_complex);

    template <typename Node>
    void arithmetic_binary_operator(Node node,
                                    BinaryOpCreator create_integer,
                                    BinaryOpCreator create_real,
                                    BinaryOpCreator create_complex);

    void visit(const Nodecl::Add &node);
    void visit(const Nodecl::Minus &node);
    void visit(const Nodecl::Mul &node);
    void visit(const Nodecl::Div &node);
    void visit(const Nodecl::Power &node);

    template <typename Node,
              typename CreateSInt,
              typename CreateFloat,
              typename CreateComplex,
              typename CreateCharacter>
    void binary_comparison(const Node node,
                           CreateSInt create_sint,
                           CreateFloat create_float,
                           CreateComplex create_complex,
                           CreateCharacter create_character);

    struct CharacterCompareLT
    {
      private:
        FortranLLVM *llvm_visitor;
        Nodecl::NodeclBase node;

      public:
        CharacterCompareLT(FortranLLVM *llvm_visitor, Nodecl::NodeclBase node);

        llvm::Value *operator()(Nodecl::NodeclBase lhs,
                                Nodecl::NodeclBase rhs,
                                llvm::Value *vlhs,
                                llvm::Value *vrhs);
    };

    struct CharacterCompareLE
    {
      private:
        FortranLLVM *llvm_visitor;
        Nodecl::NodeclBase node;

      public:
        CharacterCompareLE(FortranLLVM *llvm_visitor, Nodecl::NodeclBase node);

        llvm::Value *operator()(Nodecl::NodeclBase lhs,
                                Nodecl::NodeclBase rhs,
                                llvm::Value *vlhs,
                                llvm::Value *vrhs);
    };

    void visit(const Nodecl::LowerThan &node);
    void visit(const Nodecl::LowerOrEqualThan &node);
    void visit(const Nodecl::GreaterThan &node);
    void visit(const Nodecl::GreaterOrEqualThan &node);

    struct CharacterCompareEQ
    {
      private:
        FortranLLVM *llvm_visitor;
        Nodecl::NodeclBase node;

      public:
        CharacterCompareEQ(FortranLLVM *llvm_visitor, Nodecl::NodeclBase node);

        llvm::Value *operator()(Nodecl::NodeclBase lhs,
                                Nodecl::NodeclBase rhs,
                                llvm::Value *vlhs,
                                llvm::Value *vrhs);
    };

    void visit(const Nodecl::Equal &node);
    void visit(const Nodecl::Different &node);

    void visit(const Nodecl::Concat &node);
    // void visit(const Nodecl::ClassMemberAccess& node);

    void visit(const Nodecl::Range &node);


    // FIXME - Use GEP when possible
    llvm::Value *address_of_subscripted_array_no_descriptor(
        const Nodecl::ArraySubscript &node);

    llvm::Value *address_of_subscripted_array_descriptor(
        const Nodecl::ArraySubscript &node);

    void visit(const Nodecl::ArraySubscript &node);
    void visit(const Nodecl::FunctionCall &node);

  private:
    typedef void (FortranVisitorLLVMExpression::*BuiltinImplFunc)(
        const Nodecl::FunctionCall &node);

    struct BuiltinImpl
    {
        BuiltinImplFunc func = nullptr;
        bool use_constant = false;
    };

    // The definition is in codegen-fortran-llvm-builtin.cpp
    static std::map<std::string, BuiltinImpl> builtin_impl;

    void implement_builtin_call(const Nodecl::FunctionCall &node);

    // FIXME: Move these to another class
    void builtin_xbound(const Nodecl::FunctionCall &node, bool is_lbound);
    void builtin_lbound(const Nodecl::FunctionCall &node);
    void builtin_ubound(const Nodecl::FunctionCall &node);
};
}

#endif // CODEGEN_FORTRAN_LLVM_EXPR_HPP

