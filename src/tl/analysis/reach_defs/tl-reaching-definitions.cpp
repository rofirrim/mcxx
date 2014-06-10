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

#include "cxx-cexpr.h"

#include "tl-analysis-utils.hpp"
#include "tl-reaching-definitions.hpp"

namespace TL {
namespace Analysis {

    // **************************************************************************************************** //
    // ************************** Class implementing reaching definition analysis ************************* //

    ReachingDefinitions::ReachingDefinitions(ExtensibleGraph* graph)
            : _graph(graph)
    {}

    void ReachingDefinitions::compute_reaching_definitions()
    {
        Node* graph = _graph->get_graph();

        // Compute initial info (liveness only regarding the current node)
        gather_reaching_definitions_initial_information(graph);
        ExtensibleGraph::clear_visits(graph);

        // Common Liveness analysis
        solve_reaching_definition_equations(graph);
        ExtensibleGraph::clear_visits(graph);
    }

    void ReachingDefinitions::gather_reaching_definitions_initial_information(Node* current)
    {
        if(!current->is_visited())
        {
            current->set_visited(true);

            if(!current->is_exit_node())
            {
                if(current->is_graph_node())
                {
                    Node* entry = current->get_graph_entry_node();
                    gather_reaching_definitions_initial_information(entry);
                    set_graph_node_reaching_definitions(current);
                }
                else if(!current->is_entry_node())
                {
                    GeneratedStatementsVisitor rdv;
                    NodeclList stmts = current->get_statements();
                    for(NodeclList::iterator it = stmts.begin(); it != stmts.end(); ++it)
                    {
                        rdv.walk(*it);
                    }
                    current->set_generated_stmts(rdv.get_gen());
                }

                ObjectList<Edge*> exit_edges = current->get_exit_edges();
                for(ObjectList<Edge*>::iterator it = exit_edges.begin(); it != exit_edges.end(); ++it)
                {
                    gather_reaching_definitions_initial_information((*it)->get_target());
                }
            }
        }
    }

    void ReachingDefinitions::solve_reaching_definition_equations(Node* current)
    {
        bool changed = true;
        while(changed)
        {
            changed = false;
            solve_reaching_definition_equations_rec(current, changed);
            ExtensibleGraph::clear_visits(current);
        }
    }

    void ReachingDefinitions::solve_reaching_definition_equations_rec(Node* current, bool& changed)
    {
        if (!current->is_visited())
        {
            current->set_visited(true);

            if(!current->is_exit_node())
            {
                if(current->is_graph_node())
                {
                    solve_reaching_definition_equations_rec(current->get_graph_entry_node(), changed);
                    set_graph_node_reaching_definitions(current);
                }
                else if(!current->is_entry_node())
                {
                    NodeclMap old_rd_in = current->get_reaching_definitions_in();
                    NodeclMap old_rd_out = current->get_reaching_definitions_out();
                    NodeclMap rd_out, rd_in, pred_rd_out;

                    // Computing Reach Defs In
                    ObjectList<Node*> parents = current->get_parents();
                    for(ObjectList<Node*>::iterator it = parents.begin(); it != parents.end(); ++it)
                    {
                        bool parent_is_entry = (*it)->is_entry_node();
                        if(parent_is_entry)
                        {
                            // Iterate over outer parents while we found an ENTRY node
                            Node* entry_outer_node = (*it)->get_outer_node();
                            ObjectList<Node*> outer_parents;
                            while(parent_is_entry)
                            {
                                outer_parents = entry_outer_node->get_parents();
                                parent_is_entry = (outer_parents.size() == 1) && outer_parents[0]->is_entry_node();
                                entry_outer_node = (parent_is_entry ? outer_parents[0]->get_outer_node() : NULL);
                            }
                            // Get the Reach Def Out of the current predecessors
                            for(ObjectList<Node*>::iterator itop = outer_parents.begin(); itop != outer_parents.end(); ++itop)
                            {
                                NodeclMap outer_rd_out = (*itop)->get_reaching_definitions_out();
                                pred_rd_out.insert(outer_rd_out.begin(), outer_rd_out.end());
                            }
                        }
                        else
                        {
                            pred_rd_out = (*it)->get_reaching_definitions_out();
                        }
                        rd_in = Utils::nodecl_map_union(rd_in, pred_rd_out);
                    }

                    // Computing Reach Defs Out
                    NodeclMap gen = current->get_generated_stmts();
                    NodeclSet killed = current->get_killed_vars();
                    NodeclMap diff = Utils::nodecl_map_minus_nodecl_set(rd_in, killed);
                    
                    rd_out = Utils::nodecl_map_union(gen, diff);

                    if (!Utils::nodecl_map_equivalence(old_rd_in, rd_in) || 
                        !Utils::nodecl_map_equivalence(old_rd_out, rd_out))
                    {
                        current->set_reaching_definitions_in(rd_in);
                        current->set_reaching_definitions_out(rd_out);
                        changed = true;
                    }
                }

                ObjectList<Node*> children = current->get_children();
                for(ObjectList<Node*>::iterator it = children.begin(); it != children.end(); ++it)
                {
                    solve_reaching_definition_equations_rec(*it, changed);
                }
            }
        }
    }

    void ReachingDefinitions::set_graph_node_reaching_definitions(Node* current)
    {
        if(current->is_graph_node())
        {
            // RDI(graph) = U RDO (Y), for all Y predecessors of X
            NodeclMap graph_rdi;
            ObjectList<Node*> parents = current->get_parents();
            for(ObjectList<Node*>::iterator it = parents.begin(); it != parents.end();)
            {
                Node* c_it = *it;
                while(c_it->is_entry_node())
                {
                    if(c_it->get_outer_node()->get_id() != 0)
                        c_it = c_it->get_outer_node()->get_parents()[0];
                    else
                        goto iterate;
                }
                graph_rdi = Utils::nodecl_map_union(graph_rdi, c_it->get_reaching_definitions_out());
iterate:        ++it;
            }
            current->set_reaching_definitions_in(graph_rdi);

            // RDO(graph) = U RDO(inner exits)
            NodeclMap graph_rdo;
            ObjectList<Node*> exits = current->get_graph_exit_node()->get_parents();
            for(ObjectList<Node*>::iterator it = exits.begin(); it != exits.end(); ++it)
            {
                graph_rdo = Utils::nodecl_map_union(graph_rdo, (*it)->get_reaching_definitions_out());
            }
            if(graph_rdo.empty())
            {   // This may happen when no Reaching Defintion has been computed inside the graph or
                // when there is no statement inside the task and the information has not been propagated 
                // (Entry and Exit nodes do not contain any analysis information)
                // In this case, we propagate the Reaching Definition Out from the parents
                graph_rdo = graph_rdi;
            }
            current->set_reaching_definitions_out(graph_rdo);
        }
    }

    // *********************** End class implementing reaching definitions analysis *********************** //
    // **************************************************************************************************** //



    // **************************************************************************************************** //
    // ************************ Class implementing a visitor of reaching definition *********************** //

    GeneratedStatementsVisitor::GeneratedStatementsVisitor()
            : _gen()
    {}

    NodeclMap GeneratedStatementsVisitor::get_gen()
    {
        return _gen;
    }

    void GeneratedStatementsVisitor::visit_assignment(const NBase& lhs, const NBase& rhs)
    {
        if(_gen.find(lhs) != _gen.end())
        {   // Generated set only contains downwards exposed definitions
            // So, if there is a previous statement that generated a definition for the same variable
            // we remove it from the list
            _gen.erase(lhs);
        }
        _gen.insert(std::pair<NBase, NBase>(lhs, rhs));
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::AddAssignment& n)
    {
        Nodecl::Add rhs = Nodecl::Add::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                             n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::ArithmeticShrAssignment& n)
    {
        Nodecl::ArithmeticShr rhs = Nodecl::ArithmeticShr::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                                                 n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::Assignment& n)
    {
        visit_assignment(n.get_lhs(), n.get_rhs());
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::BitwiseAndAssignment& n)
    {
        Nodecl::BitwiseAnd rhs = Nodecl::BitwiseAnd::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                                          n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::BitwiseOrAssignment& n)
    {
        Nodecl::BitwiseOr rhs = Nodecl::BitwiseOr::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                                        n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::BitwiseShlAssignment& n)
    {
        Nodecl::BitwiseShl rhs = Nodecl::BitwiseShl::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                                          n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::BitwiseShrAssignment& n)
    {
        Nodecl::BitwiseShr rhs = Nodecl::BitwiseShr::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                                          n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::BitwiseXorAssignment& n)
    {
        Nodecl::BitwiseXor rhs = Nodecl::BitwiseXor::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                                          n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::DivAssignment& n)
    {
        Nodecl::Div rhs = Nodecl::Div::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                            n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::MinusAssignment& n)
    {
        Nodecl::Minus rhs = Nodecl::Minus::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                                 n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::ModAssignment& n)
    {
        Nodecl::Mod rhs = Nodecl::Mod::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                            n.get_type(),  n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::MulAssignment& n)
    {
        Nodecl::Mul rhs = Nodecl::Mul::make(n.get_lhs().shallow_copy(), n.get_rhs().shallow_copy(),
                                            n.get_type(), n.get_locus());
        visit_assignment(n.get_lhs(), rhs);
    }
    
    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::ObjectInit& n)
    {
        TL::Symbol lhs_sym = n.get_symbol();
        Nodecl::Symbol lhs = Nodecl::Symbol::make(lhs_sym, n.get_locus());
        NBase rhs = lhs_sym.get_value();
        
        // check for nested assignments
        walk(rhs);
        
        _gen.insert(std::pair<NBase, NBase>(lhs, rhs));
    }
    
    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::Postdecrement& n)
    {
        Nodecl::IntegerLiteral one = Nodecl::IntegerLiteral::make(n.get_type(), const_value_get_one(/* bytes */ 4, /* signed */ 1),
                                                                  n.get_locus());
        Nodecl::Minus rhs = Nodecl::Minus::make(n.get_rhs().shallow_copy(), one , n.get_type(),
                                                n.get_locus());
        visit_assignment(n.get_rhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::Postincrement& n)
    {
        Nodecl::IntegerLiteral one = Nodecl::IntegerLiteral::make(n.get_type(), const_value_get_one(/* bytes */ 4, /* signed */ 1),
                                                                   n.get_locus());
        Nodecl::Add rhs = Nodecl::Add::make(n.get_rhs().shallow_copy(), one, n.get_type(),
                                             n.get_locus());
        visit_assignment(n.get_rhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::Predecrement& n)
    {
        Nodecl::IntegerLiteral one = Nodecl::IntegerLiteral::make(n.get_type(), const_value_get_one(/* bytes */ 4, /* signed */ 1),
                                                                   n.get_locus());
        Nodecl::Minus rhs = Nodecl::Minus::make(n.get_rhs().shallow_copy(), one, n.get_type(),
                                                 n.get_locus());
        visit_assignment(n.get_rhs(), rhs);
    }

    GeneratedStatementsVisitor::Ret GeneratedStatementsVisitor::visit(const Nodecl::Preincrement& n)
    {
        Nodecl::IntegerLiteral one = Nodecl::IntegerLiteral::make(n.get_type(), const_value_get_one(/* bytes */ 4, /* signed */ 1),
                                                                   n.get_locus());
        Nodecl::Add rhs = Nodecl::Add::make(n.get_rhs().shallow_copy(), one, n.get_type(),
                                             n.get_locus());
        visit_assignment(n.get_rhs(), rhs);
    }

    // ********************** END class implementing a visitor of reaching definition ********************* //
    // **************************************************************************************************** //
}
}
