/*--------------------------------------------------------------------
(C) Copyright 2006-2009 Barcelona Supercomputing Center 
Centro Nacional de Supercomputacion

This file is part of Mercurium C/C++ source-to-source compiler.

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


#ifndef TL_CFG_VISITOR_HPP
#define TL_CFG_VISITOR_HPP

#include "tl-extensible-graph.hpp"
#include "tl-nodecl-visitor.hpp"

namespace TL
{
    struct loop_control_nodes {
        Node* init;
        Node* cond;
        Node* next;
        
        loop_control_nodes() 
            : init(NULL), cond(NULL), next(NULL) 
        {}
    };
    
    struct switch_nodes {
        Node* cond;
        Node* case_no_break;
        ObjectList<Node*> cases_break;
        int ncases;
        bool last_case_no_break;
        
        switch_nodes() 
            : cond(NULL), case_no_break(NULL), cases_break(), ncases(-1), last_case_no_break(true)
        {}
        
        void clear()
        {
            cond = NULL;
            case_no_break = NULL;
            cases_break.clear();
            ncases = -1;
            last_case_no_break = true;
        }
    };
    
    struct try_block_nodes {
        ObjectList<Node*> handler_parents;
        ObjectList<Node*> handler_exits;
        int nhandlers;
        
        try_block_nodes()
            : handler_parents(), handler_exits(), nhandlers(-1)
        {}
        
        void clear()
        {
            handler_parents.clear();
            handler_exits.clear();
            nhandlers = -1;
        }
    };
    
    class LIBTL_CLASS CfgVisitor : public Nodecl::NodeclVisitor<Node*>
    {
    protected:
        ExtensibleGraph* _actual_cfg;
        ScopeLink _sl;
        struct loop_control_nodes _actual_loop_info;
        struct switch_nodes _actual_switch_info;
        struct try_block_nodes _actual_try_info;
        
    private:
        ObjectList<ExtensibleGraph*> _cfgs;
        ObjectList<Nodecl::NodeclBase> _seq_nodecl;
        
        //! This method creates a node from a expression
        /*!
         * \param n Source expression
         * \param connect_node Boolean indicating whether the node must be connected with the last nodes
         *                     created in the graph
         * \return The new node created
         */
//         Node* get_expression_node(const Nodecl::NodeclBase& n, bool connect_node = true);
        
        //! This method creates a list with the nodes in an specific subgraph
        /*!
         * \param node First node to be traversed. The method will visit all nodes from here.
         */
//         void compute_catch_parents(Node* node);
        
        //! This method merges two nodes containing an Expression into one
        /*!
         * The way the method merges the nodes depends on the kind of the nodes to be merged:
         * The nodes that are not a GRAPH NODE are deleted. The rest, remains there to be the parents of the new node.
         * \param n Nodecl containing a Expression which will be wrapped in the new node
         * \param first Pointer to the node containing one part of the new node
         * \param second Pointer to the node containing other part of the new node
         * \return The new node created
         */
        Node* merge_nodes(Nodecl::NodeclBase n, Node* first, Node* second);
        
        //! This method finds the parent node in a sequence of connected nodes
        /*!
         * The method fails when the sub-graph has more than one entry node.
         * Since this method is used specifically to collapse the nodes created while building the node of an expression
         * we expect to find only one entry node. 
         * (We don't refer a node of BASIC_ENTRY_NODE type, but the first node in the sub-graph)
         * \param actual_node Node we are computing in this moment
         * \return The entry node of a sub-graph
         */
        Node* get_first_node(Node* actual_node);
        
    public:
        CfgVisitor(ScopeLink sl);
        CfgVisitor(const CfgVisitor& visitor);
        
        Ret unhandled_node(const Nodecl::NodeclBase& n);
        Ret visit(const Nodecl::TopLevel& n);
        Ret visit(const Nodecl::FunctionCode& n);
        Ret visit(const Nodecl::TryBlock& n);
        Ret visit(const Nodecl::CatchHandler& n);
        Ret visit(const Nodecl::Throw& n);
        Ret visit(const Nodecl::CompoundStatement& n);
        Ret visit(const Nodecl::Symbol& n);
        Ret visit(const Nodecl::ExpressionStatement& n);
        Ret visit(const Nodecl::ParenthesizedExpression& n);
        Ret visit(const Nodecl::ErrExpr& n);
        Ret visit(const Nodecl::ObjectInit& n);
        Ret visit(const Nodecl::ArraySubscript& n);
        Ret visit(const Nodecl::ClassMemberAccess& n);
        Ret visit(const Nodecl::FortranNamedPairSpec& n);
        Ret visit(const Nodecl::Concat& n);
        Ret visit(const Nodecl::New& n);
        Ret visit(const Nodecl::Delete& n);
        Ret visit(const Nodecl::DeleteArray& n);
        Ret visit(const Nodecl::Sizeof& n);
        Ret visit(const Nodecl::Type& n);
        Ret visit(const Nodecl::Typeid& n);
        Ret visit(const Nodecl::Cast& n);
        Ret visit(const Nodecl::Offset& n);
        Ret visit(const Nodecl::VirtualFunctionCall& n);
        Ret visit(const Nodecl::FunctionCall& n);
        Ret visit(const Nodecl::StringLiteral& n);
        Ret visit(const Nodecl::BooleanLiteral& n);
        Ret visit(const Nodecl::IntegerLiteral& n);
        Ret visit(const Nodecl::ComplexLiteral& n);
        Ret visit(const Nodecl::FloatingLiteral& n);
        Ret visit(const Nodecl::StructuredLiteral& n);
        Ret visit(const Nodecl::EmptyStatement& n);
        Ret visit(const Nodecl::ReturnStatement& n);
        Ret visit(const Nodecl::BuiltinExpr& n);
        Ret visit(const Nodecl::BuiltinDecl& n);
        Ret visit(const Nodecl::PragmaCustomDirective& n);
        Ret visit(const Nodecl::PragmaCustomConstruct& n);
        Ret visit(const Nodecl::PragmaCustomClause& n);
        Ret visit(const Nodecl::PragmaCustomLine& n);
        Ret visit(const Nodecl::PragmaClauseArg& n);
        Ret visit(const Nodecl::ForStatement& n);
        Ret visit(const Nodecl::WhileStatement& n);
        Ret visit(const Nodecl::IfElseStatement& n);
        Ret visit(const Nodecl::SwitchStatement& n);
        Ret visit(const Nodecl::CaseStatement& n);
        Ret visit(const Nodecl::DefaultStatement& n);
        Ret visit(const Nodecl::ConditionalExpression& n);
        Ret visit(const Nodecl::FortranComputedGotoStatement& n);
        Ret visit(const Nodecl::FortranAssignedGotoStatement& n);
        Ret visit(const Nodecl::GotoStatement& n);
        Ret visit(const Nodecl::LabeledStatement& n);
        Ret visit(const Nodecl::LoopControl& n);
        Ret visit(const Nodecl::ContinueStatement& n);
        Ret visit(const Nodecl::BreakStatement& n);
        Ret visit(const Nodecl::DoStatement& n);
        Ret visit(const Nodecl::Assignment& n);
        Ret visit(const Nodecl::AddAssignment& n);
        Ret visit(const Nodecl::SubAssignment& n);
        Ret visit(const Nodecl::DivAssignment& n);
        Ret visit(const Nodecl::MulAssignment& n);
        Ret visit(const Nodecl::ModAssignment& n);
        Ret visit(const Nodecl::BitwiseAndAssignment& n);
        Ret visit(const Nodecl::BitwiseOrAssignment& n);
        Ret visit(const Nodecl::BitwiseXorAssignment& n);
        Ret visit(const Nodecl::ShrAssignment& n);
        Ret visit(const Nodecl::ShlAssignment& n);
        Ret visit(const Nodecl::Add& n);
        Ret visit(const Nodecl::Minus& n);
        Ret visit(const Nodecl::Mul& n);
        Ret visit(const Nodecl::Div& n);
        Ret visit(const Nodecl::Mod& n);
        Ret visit(const Nodecl::Power& n);
        Ret visit(const Nodecl::LogicalAnd& n);
        Ret visit(const Nodecl::LogicalOr& n);
        Ret visit(const Nodecl::BitwiseAnd& n);
        Ret visit(const Nodecl::BitwiseOr& n);
        Ret visit(const Nodecl::BitwiseXor& n);
        Ret visit(const Nodecl::Equal& n);
        Ret visit(const Nodecl::Different& n);
        Ret visit(const Nodecl::LowerThan& n);
        Ret visit(const Nodecl::GreaterThan& n);
        Ret visit(const Nodecl::LowerOrEqualThan& n);
        Ret visit(const Nodecl::GreaterOrEqualThan& n);
        Ret visit(const Nodecl::Shr& n);
        Ret visit(const Nodecl::Shl& n);
        Ret visit(const Nodecl::Predecrement& n);
        Ret visit(const Nodecl::Postdecrement& n);
        Ret visit(const Nodecl::Preincrement& n);
        Ret visit(const Nodecl::Postincrement& n);
        Ret visit(const Nodecl::Plus& n);
        Ret visit(const Nodecl::Neg& n);     
        Ret visit(const Nodecl::BitwiseNot& n);
        Ret visit(const Nodecl::LogicalNot& n);
        Ret visit(const Nodecl::Derreference& n);
        Ret visit(const Nodecl::Reference& n);
        Ret visit(const Nodecl::Text& n);
        Ret visit(const Nodecl::FortranWhere& n);
        Ret visit(const Nodecl::FortranWherePair& n);
        Ret visit(const Nodecl::SubscriptTriplet& n);
        Ret visit(const Nodecl::FortranLabelAssignStatement& n);
        Ret visit(const Nodecl::FortranIoSpec& n);
        Ret visit(const Nodecl::FieldDesignator& n);
        Ret visit(const Nodecl::IndexDesignator& n);
        Ret visit(const Nodecl::FortranEquivalence& n);
        Ret visit(const Nodecl::FortranData& n);
        Ret visit(const Nodecl::FortranImpliedDo& n);
        Ret visit(const Nodecl::FortranForall& n);    
        Ret visit(const Nodecl::FortranArithmeticIfStatement& n);
        Ret visit(const Nodecl::FortranNullifyStatement& n);
        Ret visit(const Nodecl::FortranIoStatement& n);  
        Ret visit(const Nodecl::FortranOpenStatement& n);
        Ret visit(const Nodecl::FortranCloseStatement& n);
        Ret visit(const Nodecl::FortranReadStatement& n);
        Ret visit(const Nodecl::FortranWriteStatement& n);
        Ret visit(const Nodecl::FortranPrintStatement& n);
        Ret visit(const Nodecl::FortranStopStatement& n);
        Ret visit(const Nodecl::FortranAllocateStatement& n);
        Ret visit(const Nodecl::FortranDeallocateStatement& n);
        Ret visit(const Nodecl::Comma& n);
    };
}
#endif  // TL_CFG_VISITOR_HPP


//         Ret visit(const Nodecl::VirtualFunctionCall& n);
//         Ret visit(const Nodecl::FunctionCall& n);
//         Ret visit(const Nodecl::ConditionalExpression& n);
//         Ret visit(const Nodecl::StringLiteral& n);
//         Ret visit(const Nodecl::BooleanLiteral& n);
//         Ret visit(const Nodecl::IntegerLiteral& n);
//         Ret visit(const Nodecl::ComplexLiteral& n);
//         Ret visit(const Nodecl::FloatingLiteral& n);
//         Ret visit(const Nodecl::StructuredLiteral& n);
//         Ret visit(const Nodecl::Symbol& n);
//         Ret visit(const Nodecl::VirtualFunctionCall& n);
//         Ret visit(const Nodecl::FunctionCall& n);
//         Ret visit(const Nodecl::ArraySubscript& n);
//         Ret visit(const Nodecl::ClassMemberAccess& n);
//         Ret visit(const Nodecl::Plus& n);
//         Ret visit(const Nodecl::Neg& n);
//         Ret visit(const Nodecl::Add& n);
//         Ret visit(const Nodecl::Minus& n);
//         Ret visit(const Nodecl::Mul& n);
//         Ret visit(const Nodecl::Div& n);
//         Ret visit(const Nodecl::Mod& n);
//         Ret visit(const Nodecl::Power& n);
//         Ret visit(const Nodecl::Concat& n);
//         Ret visit(const Nodecl::Equal& n);
//         Ret visit(const Nodecl::Different& n);
//         Ret visit(const Nodecl::LowerThan& n);
//         Ret visit(const Nodecl::GreaterThan& n);
//         Ret visit(const Nodecl::LowerOrEqualThan& n);
//         Ret visit(const Nodecl::GreaterOrEqualThan& n);
//         Ret visit(const Nodecl::LogicalAnd& n);
//         Ret visit(const Nodecl::LogicalOr& n);
//         Ret visit(const Nodecl::LogicalNot& n);
//         Ret visit(const Nodecl::BitwiseAnd& n);
//         Ret visit(const Nodecl::BitwiseOr& n);
//         Ret visit(const Nodecl::BitwiseXor& n);
//         Ret visit(const Nodecl::BitwiseNot& n);
//         Ret visit(const Nodecl::Shr& n);
//         Ret visit(const Nodecl::Shl& n);
//         Ret visit(const Nodecl::Assignment& n);
//         Ret visit(const Nodecl::AddAssignment& n);
//         Ret visit(const Nodecl::SubAssignment& n);
//         Ret visit(const Nodecl::DivAssignment& n);
//         Ret visit(const Nodecl::MulAssignment& n);
//         Ret visit(const Nodecl::ModAssignment& n);        
//         Ret visit(const Nodecl::BitwiseAndAssignment& n);
//         Ret visit(const Nodecl::BitwiseOrAssignment& n);
//         Ret visit(const Nodecl::BitwiseXorAssignment& n);
//         Ret visit(const Nodecl::ShrAssignment& n);
//         Ret visit(const Nodecl::ShlAssignment& n);        
//         Ret visit(const Nodecl::ParenthesizedExpression& n);
//         Ret visit(const Nodecl::Reference& n);
//         Ret visit(const Nodecl::Derreference& n);
//         Ret visit(const Nodecl::Cast& n);
//         Ret visit(const Nodecl::ConditionalExpression& n);
//         Ret visit(const Nodecl::Comma& n);
//         Ret visit(const Nodecl::Throw& n);
//         Ret visit(const Nodecl::Predecrement& n);
//         Ret visit(const Nodecl::Postdecrement& n);
//         Ret visit(const Nodecl::Preincrement& n);
//         Ret visit(const Nodecl::Postincrement& n);
//         Ret visit(const Nodecl::Sizeof& n);    
//         Ret visit(const Nodecl::Typeid& n);
//         Ret visit(const Nodecl::Offset& n); 
//         Ret visit(const Nodecl::New& n);
//         Ret visit(const Nodecl::Delete& n);
//         Ret visit(const Nodecl::DeleteArray& n);
//         Ret visit(const Nodecl::CxxRaw& n);
//         Ret visit(const Nodecl::FortranData& n);
//         Ret visit(const Nodecl::FortranEquivalence& n);
//         Ret visit(const Nodecl::ImpliedDo& n);
//         Ret visit(const Nodecl::ObjectInit& n);
//         Ret visit(const Nodecl::BuiltinExpr& n);
//         Ret visit(const Nodecl::Text& n);
//         Ret visit(const Nodecl::ErrExpr& n);
