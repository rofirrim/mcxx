/*--------------------------------------------------------------------
  (C) Copyright 2006-2012 Barcelona Supercomputing Center
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


#include "cxx-process.h"

#include "tl-pcfg-visitor.hpp"

namespace TL {
namespace Analysis {

    // ************************************************************************************** //
    // ************************************ Static methods ********************************** //

    static bool pragma_is_worksharing(std::string pragma);

    // ********************************** END static methods ******************************** //
    // ************************************************************************************** //



    // ************************************************************************************** //
    // ************************************ Constructors ************************************ //

    PCFGVisitor::PCFGVisitor( std::string name, Scope context, bool inline_pcfg, PCFGVisitUtils* utils )
        : _pcfg( new ExtensibleGraph( name, context, utils ) ), _inline( inline_pcfg ), _utils( utils ),
          _context_s( ), _pragma_info_s( ), _omp_sections_info( ),
          _switch_cond_s( ), _visited_functions( )
    {}

    // ********************************** END constructors ********************************** //
    // ************************************************************************************** //



    // ************************************************************************************** //
    // ******************************** Non-visiting methods ******************************** //

    void PCFGVisitor::set_actual_pcfg( ExtensibleGraph* graph )
    {
        _pcfg = graph;
    }

    ExtensibleGraph* PCFGVisitor::build_pcfg( const Nodecl::NodeclBase& n )
    {
        // Visit the nodes in \n
        walk( n );

        // Complete the exit node
        Node* pcfg_exit = _pcfg->_graph->get_graph_exit_node( );
        pcfg_exit->set_id( ++_utils->_nid );

        // Connect the exit nodes to the exit node of the current graph
        _pcfg->connect_nodes( _utils->_last_nodes, pcfg_exit );
        _pcfg->connect_nodes( _utils->_return_nodes, pcfg_exit );
        _utils->_return_nodes.clear( );

        _pcfg->dress_up_graph( );

        return _pcfg;
    }

    // ****************************** END non-visiting methods ****************************** //
    // ************************************************************************************** //



    // ************************************************************************************** //
    // *********************************** Static methods *********************************** //

    static bool pragma_is_worksharing(std::string pragma)
    {
        return (pragma == "parallel" || pragma == "for" || pragma == "parallel for"
        || pragma == "workshare" || pragma == "parallel workshare"
        || pragma == "sections" || pragma == "parallel sections"
        || pragma == "single");
    }

    // ********************************* END static methods ********************************* //
    // ************************************************************************************** //



    // ************************************************************************************** //
    // ******************************** Non visiting methods ******************************** //

    void PCFGVisitor::compute_catch_parents( Node* node )
    {
        while( !node->is_visited( ) )
        {
            node->set_visited( true );
            Node_type n_type = node->get_type( );
            if( n_type == GRAPH_NODE )
            {
                compute_catch_parents( node->get_graph_entry_node( ) );
            }
            else if( n_type == BASIC_EXIT_NODE )
            {
                return;
            }
            else if( n_type != BASIC_ENTRY_NODE && n_type != UNCLASSIFIED_NODE
                     && n_type != BASIC_BREAK_NODE )
            {
                _utils->_tryblock_nodes.back( )->_handler_parents.append( node );
            }

            ObjectList<Edge*> exit_edges = node->get_exit_edges( );
            for( ObjectList<Edge*>::iterator it = exit_edges.begin( );
                 it != exit_edges.end(); it++ )
            {
                compute_catch_parents( ( *it )->get_target( ) );
            }
        }
    }

    ObjectList<Node*> PCFGVisitor::get_first_nodes(Node* actual_node)
    {
        ObjectList<Edge*> actual_entries = actual_node->get_entry_edges();
        ObjectList<Node*> actual_parents;

        if(actual_entries.empty())
        {
            if(actual_node->get_type() == BASIC_ENTRY_NODE)
            {   // 'actual_node' parent path is already connected with the graph Entry Node
            return ObjectList<Node*>();
            }
            else
            {
                return ObjectList<Node*>(1, actual_node);
            }
        }
        else
        {
            for(ObjectList<Edge*>::iterator it = actual_entries.begin();
                 it != actual_entries.end();
            ++it)
                 {
                     ObjectList<Node*> parents = get_first_nodes((*it)->get_source());
                     actual_parents.insert(parents);
                 }
        }

        return actual_parents;
    }

    /*! Elements in the list 'nodes_l' may have relations between them
     * For example, the statement 'f(b) + g();' will generate:
     * - two graph nodes which will depend one on the other
     * - the 'result' node containing the whole expression
     * This last node must have as parent only the graph node containing 'g'
     * So, before iterate the list to get the parents of the new merging node
     * we are going to purge the list deleting those nodes depending on other nodes in the same list
     */
    Node* PCFGVisitor::merge_nodes( Nodecl::NodeclBase n, ObjectList<Node*> nodes_l )
    {
        Node* result;

        // Compute the type of node for the new merged node
        Node_type ntype;
        if( n.is<Nodecl::FunctionCall>( ) || n.is<Nodecl::VirtualFunctionCall>( ) )
        {
            ntype = BASIC_FUNCTION_CALL_NODE;
        }
        else if( n.is<Nodecl::LabeledStatement>( ) )
        {
            ntype = BASIC_LABELED_NODE;
        }
        else
        {
            ntype = BASIC_NORMAL_NODE;
        }

        if( nodes_l.size() > 1
            || ( ( nodes_l.size( ) == 1 ) && ( nodes_l[0]->get_type( ) == GRAPH_NODE ) ) )
        {   // There is some node to merge. Otherwise, we only have to create the new node

        // Check whether we need to build a graph node
        bool need_graph = false;
        for( ObjectList<Node*>::iterator it = nodes_l.begin( ); it != nodes_l.end( ); ++it )
        {
            if( ( *it )->get_type( ) == GRAPH_NODE )
            {
                need_graph = true;
                break;
            }
        }

        if( need_graph )
        {
            bool found;

            // Build the new graph
            result = _pcfg->create_graph_node( _utils->_outer_nodes.top( ), Nodecl::NodeclBase::null( ), SPLIT_STMT);
            Node* entry = result->get_graph_entry_node( );

            // Get parents of the new graph node and delete the old connections
            // Parents of the nodes in the list without parents within the list will be parents of the new graph
            // Nodes in the list without parents in the list are disconnected from its parents and connected to the Entry
            ObjectList<Node*> graph_parents;
            ObjectList<int> list_pos_to_erase;
            int i = 0;
            for( ObjectList<Node*>::iterator it = nodes_l.begin( ); it != nodes_l.end( ); ++it )
            {
                found = false;
                ObjectList<Node*> actual_parents = ( *it )->get_parents( );
                ObjectList<Node*>::iterator iit;
                for( iit = nodes_l.begin( ); iit != nodes_l.end( ); ++iit )
                {
                    if( actual_parents.contains( *iit ) )
                    {
                        found = true;
                        break;
                    }
                }
                if( !found )
                {
                    // add node to the list of graph parent
                    graph_parents.append( ( *it )->get_parents( ) );

                    // disconnect those nodes of its parents
                    ObjectList<Node*> aux = ( *it )->get_parents( );
                    for(ObjectList<Node*>::iterator iit = aux.begin( ); iit != aux.end( ); ++iit )
                    {
                        ( *iit )->erase_exit_edge( *it );
                        ( *it )->erase_entry_edge( *iit );
                    }
                    // delete the node if it is not of Graph type, otherwise, connect it to the Entry
                    if( ( *it )->get_type( ) != GRAPH_NODE )
                    {
                        list_pos_to_erase.append( i );
                        delete ( *it );
                    }
                    else
                    {
                        _pcfg->connect_nodes( entry, *it );
                    }
                }
                i++;
            }
            if( !graph_parents.empty( ) )
            {
                int n_connects = graph_parents.size( );
                _pcfg->connect_nodes( graph_parents, result, ObjectList<Edge_type>( n_connects, ALWAYS_EDGE ),
                                      ObjectList<std::string>( n_connects, "" ) );
            }

            // Erase those positions in the list that are non-Graph nodes
            for( ObjectList<int>::reverse_iterator it = list_pos_to_erase.rbegin( );
                 it != list_pos_to_erase.rend( ); ++it )
            {
                nodes_l.erase( nodes_l.begin( ) + ( *it ) );
            }

                 // New merging node is created and connected with the nodes in the list without children within the list
            Node* merged_node = new Node( _utils->_nid, ntype, result, n );
            ObjectList<Node*> merged_parents;
            for( ObjectList<Node*>::iterator it = nodes_l.begin( ); it != nodes_l.end( ); ++it )
            {
                found = false;
                ObjectList<Node*> actual_children = ( *it )->get_children( );
                for( ObjectList<Node*>::iterator iit = nodes_l.begin( ); iit != nodes_l.end( ); ++iit )
                {
                    if( actual_children.contains( *iit ) )
                    {
                        found = true;
                        break;
                    }

                }
                if( !found )
                {
                    merged_parents.append( *it );
                }

                // now, all nodes must have the new Graph node as outer node
                ( *it )->set_outer_node( result );
            }
            _pcfg->connect_nodes( merged_parents, merged_node );

            // Connect merging node with the exit of the graph
            Node* graph_exit = result->get_graph_exit_node( );
            graph_exit->set_id( ++_utils->_nid );
            _pcfg->connect_nodes( merged_node, graph_exit );
            _utils->_outer_nodes.pop( );

            _utils->_last_nodes.clear( );
            _utils->_last_nodes.append( result );
        }
        else
        {
            // Delete the nodes and its connections
            for( ObjectList<Node*>::iterator it = nodes_l.begin( ); it != nodes_l.end( ); ++it )
            {
                ObjectList<Node*> aux = (*it)->get_parents();
                if( !aux.empty( ) )
                {
                    internal_error( "Deleting a non-graph node that have '%d' parents. Those nodes shouldn't have any parent'",
                                    aux.size( ) );
                }

                delete ( *it );
            }

            // Built the new node
            result = new Node( _utils->_nid, ntype, _utils->_outer_nodes.top( ), n );
        }
        }
        else
        {
            result = new Node(_utils->_nid, ntype, _utils->_outer_nodes.top(), n);
        }

        return result;
    }

    Node* PCFGVisitor::merge_nodes(Nodecl::NodeclBase n, Node* first, Node* second)
    {
        ObjectList<Node*> previous_nodes;

        previous_nodes.append(first);
        if(second != NULL)
        {   // Only second node must be NULL and it will be the case of unary operations
        previous_nodes.append(second);
        }

        return merge_nodes(n, previous_nodes);
    }

    Node* PCFGVisitor::merge_nodes(Node* subscripted, Node* subscript)
    {
        if(subscripted->get_data<Node_type>(_NODE_TYPE) || subscript->get_data<Node_type>(_NODE_TYPE))
        {

        }
        else
        {
            ObjectList<Nodecl::NodeclBase> lhs = subscripted->get_data<ObjectList<Nodecl::NodeclBase> >(_NODE_STMTS);
            ObjectList<Nodecl::NodeclBase> rhs = subscript->get_data<ObjectList<Nodecl::NodeclBase> >(_NODE_STMTS);
            if(lhs.size() != 1)
            {
                internal_error("Non graph subscripted value not correct. It must have just one statement", 0);
            }
            if(rhs.size() != 1)
            {
                internal_error("Non graph subscript value not correct. It must have just one statement", 0);
            }


        }
    }

    // ************************************************************************************** //
    // ******************************** Non visiting methods ******************************** //



    // ************************************************************************************** //
    // ********************************** Visiting methods ********************************** //

    PCFGVisitor::Ret PCFGVisitor::visit_unary_node( const Nodecl::NodeclBase& n,
                                                    const Nodecl::NodeclBase& rhs )
    {
        Node* right = walk( rhs )[0];
        return ObjectList<Node*>( 1, merge_nodes( n, right, NULL ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit_binary_node( const Nodecl::NodeclBase& n,
                                                     const Nodecl::NodeclBase& lhs,
                                                     const Nodecl::NodeclBase& rhs )
    {
        Node* left = walk( lhs )[0];
        Node* right = walk( rhs )[0];
        return ObjectList<Node*>( 1, merge_nodes( n, left, right ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit_case_or_default( const Nodecl::NodeclBase& case_stmt,
                                                         const Nodecl::NodeclBase& case_val )
    {
        // Build case nodes
        ObjectList<Node*> case_stmts = walk( case_stmt );

        ObjectList<Nodecl::NodeclBase> label;
        // Set the edge between the Case and the Switch condition
        if( !case_stmts.empty( ) )
        {
            Edge* e = _pcfg->connect_nodes( _switch_cond_s.top( ), case_stmts[0], CASE_EDGE );
            if( e != NULL )
            {   // The edge between the nodes did not exist previously
            label.append( case_val );
            e->set_data( _EDGE_LABEL, label );

            if( case_stmts.back( )->get_type( ) != BASIC_BREAK_NODE )
            {
                _utils->_last_nodes = ObjectList<Node*>( 1, case_stmts.back( ) );
            }
            }
            else
            {   // If the nodes where already connected, then the edge must have two labels
            ObjectList<Edge*> case_entry_edges = case_stmts[0]->get_entry_edges( );
            for( ObjectList<Edge*>::iterator it = case_entry_edges.begin( );
                it != case_entry_edges.end( ); ++it )
                {
                    if( ( *it )->get_source( )->get_id( ) == _switch_cond_s.top( )->get_id( ) )
                    {
                        e = *it;
                        break;
                    }
                }

                label.append( e->get_data<nodecl_t>( _EDGE_LABEL ) );
            label.append( case_val );
            e->set_data( _EDGE_LABEL, label );
            }
        }
        else
        {}   // The case is empty. Nothing to do

        return case_stmts;
    }

    PCFGVisitor::Ret PCFGVisitor::visit_literal_node( const Nodecl::NodeclBase& n )
    {
        Node* basic_node = new Node( _utils->_nid, BASIC_NORMAL_NODE, _utils->_outer_nodes.top( ), n );
        return ObjectList<Node*>( 1, basic_node );
    }


    template <typename T>
    PCFGVisitor::Ret PCFGVisitor::create_task_graph( const T& n )
    {
        ObjectList<Node*> previous_nodes;

        struct pragma_t actual_pragma;
        _pragma_info_s.push( actual_pragma );

        Node* task_graph_node;
        if( n.template is<Nodecl::PragmaCustomDeclaration>( ) )
        {   // We must build here a new Extensible Graph
            // FIXME We should use here a new PCFGVisitor, not create directly an ExtensibleGraph
//             ExtensibleGraph* last_pcfg = _pcfg;
//             _pcfg = new ExtensibleGraph( "pragma_" + n.get_symbol( ).get_name( ), n.retrieve_context( ) );
//
//             Symbol next_sym = n.get_symbol( );
//             if( next_sym.is_function( ) )
//             {
//                 scope_entry_t* next_sym_ = next_sym.get_internal_symbol( );
//                 Nodecl::FunctionCode func( next_sym_->entity_specs.function_code );
//                 Nodecl::Context ctx = func.get_statements( ).as<Nodecl::Context>( );
//
//                 task_graph_node = _pcfg->create_graph_node( _utils->_outer_nodes.top( ), n.get_pragma_line( ), TASK, ctx );
//                 task_graph_node->set_task_function( next_sym );
//                 int n_connects = _utils->_last_nodes.size( );
//                 _pcfg->connect_nodes( _utils->_last_nodes, task_graph_node,
//                                       ObjectList<Edge_type>( n_connects, ALWAYS_EDGE ),
//                                       ObjectList<std::string>( n_connects, "" ), /*is task*/ true );
//                 _utils->_last_nodes.clear( );
//                 _utils->_last_nodes.append( task_graph_node->get_graph_entry_node( ) );
//
//                 walk( n.get_pragma_line( ) );  // This visit computes information associated to the Task node,
//                 // but do not create any additional node
//
//                 walk( func.get_statements( ) );
//
//                 Node* task_graph_exit = task_graph_node->get_graph_exit_node( );
//                 task_graph_exit->set_id( ++( _utils->_nid ) );
//                 _pcfg->connect_nodes( _utils->_last_nodes, task_graph_exit );
//                 _utils->_outer_nodes.pop( );
//                 _pcfg->_task_nodes_l.append( task_graph_node );
//
//                 Node* graph_entry = _pcfg->_graph->get_graph_entry_node( );
//                 Node* graph_exit = _pcfg->_graph->get_graph_exit_node( );
//                 graph_exit->set_id( ++( _utils->_nid ) );
//
//                 _pcfg->connect_nodes( graph_entry, graph_exit );
//
//                 //                     _pcfg->dress_up_graph();
//
//                 // FIXME We need to add the new pcfg "_pcfg" to the list of cfgs in Analysis (singleton class)
//                 //                     _cfgs.append(_pcfg);
//
//                 _pcfg = last_pcfg;
//             }
//             else
//             {   // Nothing to do. Variable declarations do not create any graph
//                 internal_error("Pragma tasks declaration not related to a function not yet implemented", 0);
//             }
        }
        else
        {   // PragmaCustomStatement
            previous_nodes = _utils->_last_nodes;
            task_graph_node = _pcfg->create_graph_node( _utils->_outer_nodes.top( ), n.get_pragma_line( ), TASK,
                                                        _context_s.top( ) );
            int n_connects = _utils->_last_nodes.size( );
            _pcfg->connect_nodes( _utils->_last_nodes, task_graph_node,
                                  ObjectList<Edge_type>( n_connects, ALWAYS_EDGE ), ObjectList<std::string>( n_connects, "" ),
                                  /*is task*/ true );
            _utils->_last_nodes.clear( );
            _utils->_last_nodes.append( task_graph_node->get_graph_entry_node( ) );

            walk( n.get_pragma_line( ) );  // This visit computes information associated to the Task node,
                                           // but do not create any additional node

            Nodecl::PragmaCustomStatement n_stmt = n.template as<Nodecl::PragmaCustomStatement>( );
            walk( n_stmt.get_statements( ) );

            Node* graph_exit = task_graph_node->get_graph_exit_node( );
            graph_exit->set_id( ++( _utils->_nid ) );
            _pcfg->connect_nodes( _utils->_last_nodes, graph_exit );
            _utils->_outer_nodes.pop( );
            _pcfg->_task_nodes_l.append( task_graph_node );

            _utils->_last_nodes = previous_nodes;
        }

        return ObjectList<Node*>( 1, task_graph_node );
    }

    template <typename T>
    PCFGVisitor::Ret PCFGVisitor::visit_function_call( const T& n )
    {
        // Add the current Function Call to the list of called functions
        _pcfg->add_func_call_symbol( n.get_called( ).get_symbol( ) );

        // Create the new Function Call node and built it
        Node* func_graph_node = _pcfg->create_graph_node( _utils->_outer_nodes.top( ), Nodecl::NodeclBase::null( ), FUNC_CALL );
        if( !_utils->_last_nodes.empty( ) )
        {   // If there is any node in 'last_nodes' list, then we have to connect the new graph node
            _pcfg->connect_nodes( _utils->_last_nodes, func_graph_node );
            _utils->_last_nodes.clear( );
        }
        _utils->_last_nodes.append( func_graph_node->get_graph_entry_node( ) );

        // Create the nodes for the arguments
        Node* func_node;
        Nodecl::List args = n.get_arguments( ).template as<Nodecl::List>( );
        ObjectList<Node*> arguments_l = walk( args );
        if( !arguments_l.empty( ) )
        {   // Method merge_nodes connects properly the nodes created
            func_node = merge_nodes( n, arguments_l );
        }
        else
        {
            func_node = new Node( _utils->_nid, BASIC_FUNCTION_CALL_NODE, func_graph_node, n );
        }
        _pcfg->connect_nodes( _utils->_last_nodes, func_node );

        Node* graph_exit = func_graph_node->get_graph_exit_node( );
        graph_exit->set_id( ++( _utils->_nid ) );
        _pcfg->connect_nodes( func_node, graph_exit );

        _utils->_outer_nodes.pop( );
        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( func_graph_node );

        return ObjectList<Node*>( 1, func_graph_node );
    }

    template<typename T>
    PCFGVisitor::Ret PCFGVisitor::visit_pragma_construct( const T& n )
    {
        // Built a new object in the pragma stack to store its relative info
        struct pragma_t actual_pragma;
        _pragma_info_s.push( actual_pragma );

        std::string pragma = n.get_pragma_line( ).get_text( );

        if( pragma == "task" )
        {
            return create_task_graph( n );
        }
        else
        {
            if( pragma == "section" )
            {
                _utils->_last_nodes = _omp_sections_info.top( ).section_parents;
            }

            Node* pragma_graph_node = _pcfg->create_graph_node( _utils->_outer_nodes.top( ), n.get_pragma_line( ), OMP_PRAGMA );
            if( !_utils->_last_nodes.empty( ) )
            {   // If there is any node in 'last_nodes' list, then we have to connect the new graph node
                _pcfg->connect_nodes( _utils->_last_nodes, pragma_graph_node );
                _utils->_last_nodes.clear( );
            }
            _utils->_last_nodes.append( pragma_graph_node->get_graph_entry_node( ) );

            walk( n.get_pragma_line( ) );  // This visit computes information associated to the Pragma node,
            // but do not create any additional node

            if( pragma == "parallel" || pragma == "parallel for"
                 || pragma == "parallel workshare" || pragma == "parallel sections"
                 || pragma == "critical" || pragma == "atomic" || pragma == "ordered"
                 || pragma == "task" )
            {
                // We include here a Flush node before the pragma statements
                Node* flush_node = new Node( _utils->_nid, FLUSH_NODE, pragma_graph_node );
                _pcfg->connect_nodes( _utils->_last_nodes, flush_node );
                _utils->_last_nodes.clear( );
                _utils->_last_nodes.append( flush_node );
            }
            else if( pragma == "workshare" || pragma == "single" || pragma == "master"
                      || pragma == "for" || pragma == "section" )
            {   // Nothing to do before building the pragma inner statements
            }
            else
            {
                internal_error( "Unexpected omp pragma construct '%s' while building the CFG", pragma.c_str( ) );
            }

            if( pragma == "sections" || pragma == "parallel sections" )
            {   // push a new struct in the stack to store info about entries and exits
                struct omp_pragma_sections_t actual_sections_info( _utils->_last_nodes );
                _omp_sections_info.push( actual_sections_info );

                if( n.template is<Nodecl::PragmaCustomDeclaration>( ) )
                {
                    Symbol next_sym = n.get_symbol( );
                    if( next_sym.is_function( ) )
                    {
                        scope_entry_t* next_sym_ = next_sym.get_internal_symbol( );
                        // TODO
                        internal_error( "Sections pragma '%s' at declaration level. Not implemented yet", 0 );
                    }
                    else
                    {   // Nothing to do. Variable declarations do not create any graph
                    }
                }
                else
                {   // PragmaCustomStatement

                    Nodecl::PragmaCustomStatement n_stmt = n.template as<Nodecl::PragmaCustomStatement>( );
                    walk( n_stmt.get_statements( ) );    // We will not use the list result of this walk
                }
            }
            else
            {
                if( n.template is<Nodecl::PragmaCustomDeclaration>( ) )
                {
                    Symbol next_sym = n.get_symbol( );
                    if( next_sym.is_function( ) )
                    {
                        scope_entry_t* next_sym_ = next_sym.get_internal_symbol( );
                        // TODO
                        internal_error( "Non sections pragma '%s' at declaration level. Not implemented yet", 0 );
                    }
                    else
                    {   // Nothing to do. Variable declarations do not create any graph
                    }
                }
                else
                {   // PragmaCustomStatement
                    Nodecl::PragmaCustomStatement n_stmt = n.template as<Nodecl::PragmaCustomStatement>( );
                    walk( n_stmt.get_statements( ) );
                }
            }

            if( pragma == "section" )
            {
                _omp_sections_info.top( ).sections_exits.append( pragma_graph_node );
            }
            else if( pragma == "sections" || pragma == "parallel sections" )
            {
                _utils->_last_nodes = _omp_sections_info.top( ).sections_exits;
                _omp_sections_info.pop( );
            }

            if( pragma_is_worksharing( pragma ) && !_pragma_info_s.top( ).has_clause( "nowait" ) )
            {   // We include here a Barrier node after the pragma statement
                _pcfg->create_barrier_node( pragma_graph_node );
            }
            else if( pragma == "parallel" || pragma == "for" || pragma == "parallel for"
                     || pragma == "workshare" || pragma == "parallel workshare"
                     || pragma == "sections" || pragma == "parallel sections"
                     || pragma == "critical" || pragma == "atomic" || pragma == "ordered" || pragma == "single" )
            {   // This constructs add a Flush at the end of the construct
                // FIXME Atomic construct implies a list of variables to be flushed
                Node* flush_node = new Node( _utils->_nid, FLUSH_NODE, pragma_graph_node );
                _pcfg->connect_nodes( _utils->_last_nodes, flush_node );
                _utils->_last_nodes.clear( );
                _utils->_last_nodes.append( flush_node );
            }

            Node* graph_exit = pragma_graph_node->get_graph_exit_node( );
            graph_exit->set_id( ++( _utils->_nid ) );
            _pcfg->connect_nodes( _utils->_last_nodes, graph_exit );
            _utils->_outer_nodes.pop( );
            _utils->_last_nodes.clear( );
            _utils->_last_nodes.append( pragma_graph_node );

            return ObjectList<Node*>( 1, pragma_graph_node );
        }
    }

    PCFGVisitor::Ret PCFGVisitor::unhandled_node( const Nodecl::NodeclBase& n )
    {
        std::cerr << "Unhandled node while CFG construction '"
                    << codegen_to_str( n.get_internal_nodecl( ),
                                        nodecl_retrieve_context( n.get_internal_nodecl( ) ) )
                    << "' of type '" << ast_print_node_type( n.get_kind( ) ) << "'" << std::endl;
        return PCFGVisitor::Ret( );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Add& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::AddAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Alignof& n )
    {
        return visit_unary_node( n, n.get_align_type( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ArithmeticShr& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ArithmeticShrAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ArraySubscript& n )
    {
        ObjectList<Node*> subscripted = walk( n.get_subscripted( ) );
        ObjectList<Node*> subscripts = walk( n.get_subscripts( ) );

        ObjectList<Node*> nodes = subscripted;
        nodes.insert( subscripts );
        return ObjectList<Node*>( 1, merge_nodes( n, nodes ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Assignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseAnd& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseAndAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseNot& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseOr& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseOrAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseShl& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseShlAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseShr& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseShrAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseXor& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BitwiseXorAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BooleanLiteral& n )
    {
        return visit_literal_node( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::BreakStatement& n )
    {
        Node* break_node = _pcfg->append_new_node_to_parent( _utils->_last_nodes, n, BASIC_BREAK_NODE );
        _pcfg->connect_nodes( break_node, _utils->_break_nodes.top( ) );
        _utils->_last_nodes.clear( );
        return ObjectList<Node*>( 1, break_node );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::CaseStatement& n )
    {
        return visit_case_or_default( n.get_statement( ), n.get_case( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Cast& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::CatchHandler& n )
    {
        PCFGTryBlock* current_tryblock = _utils->_tryblock_nodes.back( );
        current_tryblock->_nhandlers++;

        // Build the handler nodes
        _utils->_last_nodes = current_tryblock->_handler_parents;
        ObjectList<Node*> catchs = walk( n.get_statement( ) );

        current_tryblock->_handler_exits.append( catchs[0] );

        // Set the type of the edge between each handler parent and the actual handler
        Nodecl::NodeclBase label = n.get_name( );
        for( ObjectList<Node*>::iterator it = current_tryblock->_handler_parents.begin( );
              it != current_tryblock->_handler_parents.end( ); ++it )
        {
            Edge* catch_edge = ( *it )->get_exit_edge( catchs[0] );
            if( catch_edge != NULL )
            {
                catch_edge->set_data( _EDGE_TYPE, CATCH_EDGE );
                catch_edge->set_data( _EDGE_LABEL, ObjectList<Nodecl::NodeclBase>( 1, label ) );
            }
        }

        // FIXME If there is no Ellipsis, all statements within Try must be connected to the Exit of the graph
        // TODO We can reduce considerably the number of connections by analysing the kind of every exception

        return catchs;
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ClassMemberAccess& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_member( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Comma& n )
    {
        ObjectList<Node*> comma_nodes;
        comma_nodes.append( walk( n.get_rhs( ) ) );
        comma_nodes.append( walk( n.get_lhs( ) ) );
        return comma_nodes;
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ComplexLiteral& n )
    {
        return visit_literal_node( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::CompoundStatement& n )
    {
        return walk( n.get_statements( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Concat& n )
    {
        return visit_binary_node( n, n.get_lhs(), n.get_rhs() );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ConditionalExpression& n )
    {
        Node* cond_expr_node = _pcfg->create_graph_node( _utils->_outer_nodes.top( ),
                                                         Nodecl::NodeclBase::null( ), COND_EXPR );
        Node* entry_node = cond_expr_node->get_graph_entry_node( );

        // Build condition node
        Node* condition_node = walk( n.get_condition( ) )[0];
        _pcfg->connect_nodes( entry_node, condition_node );
        ObjectList<Node*> exit_parents;

        // Build true node
        Node* true_node = walk( n.get_true( ) )[0];
        _pcfg->connect_nodes( condition_node, true_node );
        exit_parents.append( true_node );

        // Build false node
        Node* false_node = walk( n.get_false( ) )[0];
        _pcfg->connect_nodes( condition_node, false_node );
        exit_parents.append( false_node );

        // Set exit graph node info
        Node* graph_exit = cond_expr_node->get_graph_exit_node( );
        graph_exit->set_id( ++( _utils->_nid ) );
        _pcfg->connect_nodes( exit_parents, graph_exit );
        _utils->_outer_nodes.pop( );

        return ObjectList<Node*>( 1, cond_expr_node );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Context& n )
    {
        _context_s.push( n );
        ObjectList<Node*> in_context = walk( n.get_in_context( ) );
        _context_s.pop( );
        return in_context;
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ContinueStatement& n )
    {
        Node* continue_node = _pcfg->append_new_node_to_parent( _utils->_last_nodes, n, BASIC_CONTINUE_NODE );
        _pcfg->connect_nodes( continue_node, _utils->_continue_nodes.top( ) );
        _utils->_last_nodes.clear( );
        return ObjectList<Node*>( 1, continue_node );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Conversion& n )
    {
        return walk( n.get_nest( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::DefaultStatement& n )
    {
        return visit_case_or_default( n.get_statement( ), Nodecl::NodeclBase::null( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Delete& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::DeleteArray& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Dereference& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Different& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Div& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::DivAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::DoStatement& n )
    {
        // FIXME This is not correct:
        //             do
        //             {
        //             #pragma omp task private(fX) firstprivate(i)
        //             {
        //                     fX = f(fH * ((double)i + 0.5));
        //                 #pragma omp critical
        //                     fSum += fX;
        //             }
        //             #pragma omp task
        //             {
        //                 int a = 0;
        //                 a++;
        //             }
        //             i += 1;
        //             }while(i < n);

        ObjectList<Node*> do_parents = _utils->_last_nodes;

        Node* exit_node = new Node( );
        Node* aux_condition_node = new Node( );
        _utils->_continue_nodes.push( aux_condition_node );
        _utils->_break_nodes.push( exit_node );
        ObjectList<Node*> stmts = walk( n.get_statement( ) );
        _utils->_continue_nodes.pop( );
        _utils->_break_nodes.pop( );
        if( !stmts.empty( ) )
        {   // There is something within the Do Statement
            _utils->_last_nodes.clear( );
            _utils->_last_nodes.append( stmts.back( ) );
        }

        Node* condition_node = walk( n.get_condition( ) )[0];
        if( aux_condition_node->is_connected( ) )
        {
            int n_connects = aux_condition_node->get_parents( ).size( );
            _pcfg->connect_nodes( aux_condition_node->get_parents( ), condition_node, ALWAYS_EDGE, "" );
            _pcfg->connect_nodes( condition_node, aux_condition_node->get_children( ),
                                  ObjectList<Edge_type>( n_connects, ALWAYS_EDGE ),
                                  ObjectList<std::string>( n_connects, "" ) );
        }

        _pcfg->connect_nodes( stmts.back( ), condition_node );
        if( !stmts.empty( ) )
        {
            _pcfg->connect_nodes( condition_node, stmts[0], TRUE_EDGE );
        }

        // Connect the False condition side to a provisional node
        exit_node->set_id( ++( _utils->_nid ) );
        exit_node->set_outer_node( _utils->_outer_nodes.top( ) );
        _pcfg->connect_nodes( condition_node, exit_node, FALSE_EDGE );

        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( exit_node );

        if( !stmts.empty( ) )
            return ObjectList<Node*>( 1, stmts[0] );
        else
            return ObjectList<Node*>( 1, condition_node );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::EmptyStatement& n )
    {
        return visit_literal_node( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Equal& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ExpressionStatement& n )
    {
        ObjectList<Node*> expr_last_nodes = _utils->_last_nodes;
        ObjectList<Node*> expression_nodes = walk( n.get_nest( ) );

        if( expression_nodes.size( ) > 0 )
        {
            // When the size of expression_nodes list is >1, the expression contained contains a comma operator.
            // Otherwise, the expression is any other kind of expression
            Node* last_node;
            if( expression_nodes.size( ) == 1 )
            {
                last_node = expression_nodes[0];
            }
            else
            {   // expression_nodes.size() > 1
                last_node = merge_nodes( n, expression_nodes );
            }

            if( !last_node->is_empty_node( ) )
            {
                // Connect the partial node created recursively with the piece of Graph build until this moment
                ObjectList<Node*> expr_first_nodes = get_first_nodes( last_node );
                for( ObjectList<Node*>::iterator it = expr_first_nodes.begin( );
                     it != expr_first_nodes.end( ); ++it )
                {
                    _pcfg->clear_visits( *it );
                }

                if( !expr_last_nodes.empty( ) )
                {   // This will be empty when last statement visited was a Break Statement
                    int n_connects = expr_first_nodes.size( ) * expr_last_nodes.size( );
                    if( n_connects != 0 )
                    {
                        _pcfg->connect_nodes( expr_last_nodes, expr_first_nodes,
                                              ObjectList<Edge_type>( n_connects, ALWAYS_EDGE ),
                                              ObjectList<std::string>( n_connects, "" ) );
                    }
                }

                // Recompute actual last nodes for the actual graph
                if( !_utils->_last_nodes.empty( ) )
                {
                    _utils->_last_nodes.clear( );
                    _utils->_last_nodes.append( last_node );
                }
            }
            else
            {   // do nothing; this case appears when the expression is "new"
                // In this case we don't need adding this statement to the graph because it is meaningless
            }
        }
        else
        {
            internal_error( "Parsing the expression '%s' 0 nodes has been returned, and they must be one or more\n",
                            codegen_to_str( n.get_internal_nodecl( ),
                                            nodecl_retrieve_context( n.get_internal_nodecl( ) ) ) );
        }

        return expression_nodes;
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::FloatingLiteral& n )
    {
        return visit_literal_node( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ForStatement& n )
    {
        // Compute the information about the loop control and keep the results in the struct '_current_loop_ctrl'
        ObjectList<Node*> actual_last_nodes = _utils->_last_nodes;
        walk( n.get_loop_header( ) );
        _utils->_last_nodes = actual_last_nodes;

        // Connect the init
        if( _utils->_nested_loop_nodes.top( )->_init != NULL )
        {
            int n_connects = _utils->_last_nodes.size( );
            _pcfg->connect_nodes( _utils->_last_nodes, _utils->_nested_loop_nodes.top( )->_init,
                                  ObjectList<Edge_type>( n_connects, ALWAYS_EDGE ),
                                  ObjectList<std::string>( n_connects, "" ) );
            _utils->_last_nodes.clear( );
            _utils->_last_nodes.append( _utils->_nested_loop_nodes.top( )->_init );
        }

        // Create the natural loop graph node
        Node* for_graph_node = _pcfg->create_graph_node( _utils->_outer_nodes.top( ), n.get_loop_header( ),
                                                         LOOP, Nodecl::NodeclBase::null( ) );
        for_graph_node->set_loop_node_type( FOR );
        int n_connects = _utils->_last_nodes.size( );
        _pcfg->connect_nodes( _utils->_last_nodes, for_graph_node,
                              ObjectList<Edge_type>( n_connects, ALWAYS_EDGE ),
                              ObjectList<std::string>( n_connects, "" ) );

        // Connect the conditional node
        Node* entry_node = for_graph_node->get_graph_entry_node( );
        _utils->_nested_loop_nodes.top( )->_cond->set_outer_node( for_graph_node );
        _pcfg->connect_nodes( entry_node, _utils->_nested_loop_nodes.top( )->_cond );
        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( _utils->_nested_loop_nodes.top( )->_cond );

        Node* exit_node = for_graph_node->get_graph_exit_node( );

        // Create the nodes from the list of inner statements of the loop
        _utils->_continue_nodes.push( _utils->_nested_loop_nodes.top( )->_next );
        _utils->_break_nodes.push( exit_node );
        walk( n.get_statement( ) );    // This list of nodes returned here will never be used
        _utils->_continue_nodes.pop();
        _utils->_break_nodes.pop();

        // Compute the true edge from the loop condition
        Edge_type aux_etype = ALWAYS_EDGE;
        ObjectList<Edge*> exit_edges = _utils->_nested_loop_nodes.top( )->_cond->get_exit_edges( );
        if( !exit_edges.empty( ) )
        {
            // The first edge and, and, if the first is a task, all the following edges being tasks and the first not being a task are TRUE_EDGE
            // If all exit exit edges are tasks, then the "Next node" of the loop is linked with a true edge as well
            bool all_tasks = true;
            ObjectList<Edge*>::iterator it = exit_edges.begin( );
            while( all_tasks && it != exit_edges.end( ) )
            {
                if( !( *it )->is_task_edge( ) )
                {
                    all_tasks = false;
                    ( *it )->set_data( _EDGE_TYPE, TRUE_EDGE );
                }
                else
                {
                    ( *it )->set_data( _EDGE_TYPE, TRUE_EDGE );
                }
                ++it;
            }
            if( all_tasks )
                aux_etype = TRUE_EDGE;
        }
        else
        {   // It will be empty when the loop's body is empty.
            aux_etype = TRUE_EDGE;
        }

        exit_node->set_id( ++( _utils->_nid ) );
        _pcfg->connect_nodes( _utils->_nested_loop_nodes.top( )->_cond, exit_node, FALSE_EDGE );

        // Fill the empty fields of the Increment node
        _utils->_nested_loop_nodes.top( )->_next->set_outer_node( for_graph_node );
        _pcfg->connect_nodes( _utils->_last_nodes, _utils->_nested_loop_nodes.top( )->_next, aux_etype );
        _pcfg->connect_nodes( _utils->_nested_loop_nodes.top( )->_next, _utils->_nested_loop_nodes.top( )->_cond,
                              ALWAYS_EDGE, "", /* is back edge */ true );

        for_graph_node->set_stride_node( _utils->_nested_loop_nodes.top( )->_next );

        _utils->_outer_nodes.pop( );
        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( for_graph_node );

        _utils->_nested_loop_nodes.pop( );

        return ObjectList<Node*>( 1, for_graph_node );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::FunctionCall& n )
    {
        return visit_function_call( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::FunctionCode& n )
    {
        _pcfg->_function_sym = n.get_symbol( );

        return walk( n.get_statements( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::GotoStatement& n )
    {
        Node* goto_node = _pcfg->append_new_node_to_parent( _utils->_last_nodes, n, BASIC_GOTO_NODE );
        goto_node->set_label( n.get_symbol( ) );
        _pcfg->connect_nodes( _utils->_last_nodes, goto_node );

        for( ObjectList<Node*>::iterator it = _utils->_labeled_nodes.begin( );
             it != _utils->_labeled_nodes.end( ); ++it )
        {
            if( ( *it )->get_label( ) == n.get_symbol( ) )
            {   // Connect the nodes
                _pcfg->connect_nodes( goto_node, *it, GOTO_EDGE, n.get_symbol( ).get_name( ) );
                break;
            }
        }

        _utils->_last_nodes.clear( );

        return ObjectList<Node*>( 1, goto_node );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::GreaterOrEqualThan& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::GreaterThan& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::IfElseStatement& n )
    {
        Node* exit_node = new Node( );

        // Compose the condition node
        ObjectList<Node*> cond_last_nodes = _utils->_last_nodes;
        ObjectList<Node*> cond_node_l = walk( n.get_condition( ) );
        _pcfg->connect_nodes( cond_last_nodes, cond_node_l[0] );
        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( cond_node_l[0] );

        // Compose the then node
        ObjectList<Node*> then_node_l = walk( n.get_then( ) );
        Nodecl::NodeclBase then = n.get_then( );
        if( !cond_node_l[0]->get_exit_edges( ).empty( ) )
        {
            ObjectList<Edge*> exit_edges = cond_node_l[0]->get_exit_edges( );
            bool all_tasks_then = true;
            for( ObjectList<Edge*>::iterator it = exit_edges.begin( ); it != exit_edges.end( ); ++it )
            {   // More than one exit edge means that some tasks are created within 'then' statement
                ( *it )->set_data( _EDGE_TYPE, TRUE_EDGE );
                if( !( *it )->is_task_edge( ) )
                    all_tasks_then = false;
            }

            // Compose the else node, if it exists
            ObjectList<Node*> last_nodes_after_then = _utils->_last_nodes;
            _utils->_last_nodes.clear( );
            _utils->_last_nodes.append( cond_node_l[0] );
            ObjectList<Node*> else_node_l = walk( n.get_else( ) );

            // Link the If condition with the FALSE statement (else or empty node)
            bool all_tasks_else = true;
            int false_edge_it = exit_edges.size( );
            exit_edges = cond_node_l[0]->get_exit_edges( );
            for( ; false_edge_it < cond_node_l[0]->get_exit_edges( ).size( ); ++false_edge_it )
            {
                exit_edges[false_edge_it]->set_data( _EDGE_TYPE, FALSE_EDGE );
                if( !exit_edges[false_edge_it]->is_task_edge( ) )
                    all_tasks_else = false;
            }

            exit_node->set_id( ++( _utils->_nid ) );
            exit_node->set_outer_node( _utils->_outer_nodes.top( ) );

            if( ( all_tasks_then && all_tasks_else ) || ( then_node_l.empty( ) && else_node_l.empty( ) ) )
            {
                _pcfg->connect_nodes( cond_node_l[0], exit_node );
            }
            else
            {
                if( then_node_l.empty( ) )
                    _pcfg->connect_nodes( cond_node_l[0], exit_node, TRUE_EDGE );
                else if( else_node_l.empty( ) )
                    _pcfg->connect_nodes( cond_node_l[0], exit_node, FALSE_EDGE );

                if( all_tasks_else || else_node_l.empty( ) )
                    _utils->_last_nodes = last_nodes_after_then;

                _pcfg->connect_nodes( _utils->_last_nodes, exit_node );
            }
        }
        else
        {
            // Both for true and false evaluation for the if condition, we go to the exit node
            exit_node->set_id( ++( _utils->_nid ) );
            exit_node->set_outer_node( _utils->_outer_nodes.top() );
            _pcfg->connect_nodes( cond_node_l[0], exit_node, TRUE_EDGE );
            _pcfg->connect_nodes( cond_node_l[0], exit_node, FALSE_EDGE );
        }

        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( exit_node );

        return cond_node_l;
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::IntegerLiteral& n )
    {
        return visit_literal_node( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::LabeledStatement& n )
    {
        Node* labeled_node = walk( n.get_statement( ) )[0];
        labeled_node->set_data( _NODE_TYPE, BASIC_LABELED_NODE );
        labeled_node->set_label( n.get_symbol( ) );

        for( ObjectList<Node*>::iterator it = _utils->_goto_nodes.begin( );
              it != _utils->_goto_nodes.end( ); ++it )
        {
            if( ( *it )->get_label( ) == n.get_symbol( ) )
            {   // Connect the nodes
                _pcfg->connect_nodes( *it, labeled_node, GOTO_EDGE, n.get_symbol( ).get_name( ) );
                break;
            }
        }

        _utils->_labeled_nodes.append( labeled_node );
        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( labeled_node );

        return ObjectList<Node*>( 1, labeled_node );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::LogicalAnd& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::LogicalNot& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::LogicalOr& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::LoopControl& n )
    {
        PCFGLoopControl* current_loop_ctrl = new PCFGLoopControl( );

        // Create initializing node
        _utils->_last_nodes.clear( );
        ObjectList<Node*> init_node_l = walk( n.get_init( ) );
        if( init_node_l.empty( ) )
        {   // The empty statement will return anything here. No node needs to be created
            current_loop_ctrl->_init = NULL;
        }
        else
        {
            current_loop_ctrl->_init = init_node_l[0];
        }

        // Create condition node
        _utils->_last_nodes.clear( );
        ObjectList<Node*> cond_node_l = walk( n.get_cond( ) );
        if( cond_node_l.empty( ) )
        {   // The condition is an empty statement.
            // In any case, we build here a node for easiness
            current_loop_ctrl->_cond = new Node( _utils->_nid, BASIC_NORMAL_NODE,
                                              _utils->_outer_nodes.top( ), Nodecl::NodeclBase::null( ) );
        }
        else
        {
            current_loop_ctrl->_cond = cond_node_l[0];
        }

        // Create next node
        _utils->_last_nodes.clear( );
        ObjectList<Node*> next_node_l = walk( n.get_next( ) );
        if( next_node_l.empty( ) )
        {
            current_loop_ctrl->_next = new Node( _utils->_nid, BASIC_NORMAL_NODE,
                                              _utils->_outer_nodes.top( ), Nodecl::NodeclBase::null( ) );
        }
        else
        {
            current_loop_ctrl->_next = next_node_l[0];
        }

        _utils->_nested_loop_nodes.push( current_loop_ctrl );

        return PCFGVisitor::Ret( );   // No return required here. '_current_loop_ctrl' contains all information.
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::LowerOrEqualThan& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::LowerThan& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Minus& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::MinusAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Mod& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ModAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Mul& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::MulAssignment& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Neg& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::New& n )
    {
        return visit_literal_node( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ObjectInit& n )
    {
        if( _pcfg == NULL )
        {   // do nothing: A shared variable is declared
            return PCFGVisitor::Ret( );
        }
        else
        {
            ObjectList<Node*> object_init_last_nodes = _utils->_last_nodes;
            nodecl_t n_sym = nodecl_make_symbol( n.get_symbol( ).get_internal_symbol( ),
                                                 n.get_filename( ).c_str( ), n.get_line( ) );
            Nodecl::Symbol nodecl_symbol( n_sym );
            ObjectList<Node*> init_sym = walk( nodecl_symbol );
            ObjectList<Node*> init_expr = walk( n.get_symbol( ).get_value( ) );

            if( init_expr.empty( ) )
            {   // do nothing: The Object Init is not initialized
                return PCFGVisitor::Ret( );
            }

            Node* merged_node = merge_nodes( n, init_sym[0], init_expr[0] );
            _pcfg->connect_nodes( object_init_last_nodes, merged_node );
            _utils->_last_nodes.clear( );
            _utils->_last_nodes.append( merged_node );
            return ObjectList<Node*>( 1, merged_node );
        }
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Offset& n )
    {
        return visit_binary_node( n, n.get_base( ), n.get_offset( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Offsetof& n )
    {
        return visit_binary_node( n, n.get_offset_type( ), n.get_designator( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Plus& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    // FIXME This may not be correct
    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::PointerToMember& n )
    {
        // Tag the symbol if it is a global variable
        Scope s_sc = n.get_symbol( ).get_scope( );
        Nodecl::NodeclBase n2 = n;
        if( !s_sc.scope_is_enclosed_by( _pcfg->_sc ) )
        {
            Utils::ExtendedSymbolUsage glob_var_usage( n, Utils::undefined_usage );
            if( !Utils::usage_list_contains_sym( glob_var_usage.get_nodecl( ).get_symbol( ), _pcfg->_global_vars ) )
                _pcfg->_global_vars.insert( glob_var_usage );
        }

        // Create the node
        Node* basic_node = new Node( _utils->_nid, BASIC_NORMAL_NODE, _utils->_outer_nodes.top( ), n );
        return ObjectList<Node*>( 1, basic_node );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Postdecrement& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Postincrement& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Power& n )
    {
        return visit_binary_node( n, n.get_lhs( ), n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::PragmaClauseArg& n )
    {
        _pragma_info_s.top( ).clauses.back( ).args.append( n );
        return PCFGVisitor::Ret( );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::PragmaCustomClause& n )
    {
        struct clause_t actual_clause( n.get_text( ) );
        _pragma_info_s.top( ).clauses.append( actual_clause );
        walk( n.get_arguments( ) );    // This call fills _pragma_info_s with the arguments of the actual clause
        return PCFGVisitor::Ret( );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::PragmaCustomDeclaration& n )
    {
        return visit_pragma_construct( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::PragmaCustomDirective& n )
    {
        std::string pragma_line = n.get_pragma_line( ).get_text( );
        if( pragma_line == "barrier" )
        {
            _pcfg->create_barrier_node( _utils->_outer_nodes.top( ) );
        }
        else if( pragma_line == "taskwait" )
        {
            Node* taskwait_node = new Node( _utils->_nid, TASKWAIT_NODE, _utils->_outer_nodes.top( ) );
            _pcfg->connect_nodes( _utils->_last_nodes, taskwait_node );
            _utils->_last_nodes.clear( );
            _utils->_last_nodes.append( taskwait_node );
        }
        else
        {
            internal_error( "Unexpected directive '%s' found while building the CFG.", pragma_line.c_str( ) );
        }
        return PCFGVisitor::Ret( );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::PragmaCustomLine& n )
    {
        // Get the empty clause as 'parameters'
        // We create conservatively a 'clause_t'. If no arguments has been found, then we remove it
        struct clause_t actual_clause( n.get_text( ) );
        _pragma_info_s.top( ).clauses.append( actual_clause );
        walk( n.get_parameters( ) );
        if( _pragma_info_s.top( ).clauses.back( ).args.empty( ) )
        {
            _pragma_info_s.top( ).clauses.erase( _pragma_info_s.top( ).clauses.end( ) - 1 );
        }

        // Get the rest of clauses
        walk( n.get_clauses( ) );  // This method fills _pragma_info_s with the clauses of the actual pragma

        return PCFGVisitor::Ret( );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::PragmaCustomStatement& n )
    {
        return visit_pragma_construct(n);
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Predecrement& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Preincrement& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Range& n )
    {
        ObjectList<Node*> lower = walk( n.get_lower( ) );
        ObjectList<Node*> upper = walk( n.get_upper( ) );
        ObjectList<Node*> stride = walk( n.get_stride( ) );

        Node* merged_limits = merge_nodes( n, lower[0], upper[0] );
        Node* merged = merge_nodes( n, merged_limits, stride[0] );
        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append(merged);

        return ObjectList<Node*>( 1, merged );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Reference& n )
    {
        return visit_unary_node( n, n.get_rhs( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::ReturnStatement& n )
    {
        ObjectList<Node*> return_last_nodes = _utils->_last_nodes;
        ObjectList<Node*> returned_value = walk( n.get_value( ) );
        Node* return_node = merge_nodes( n, returned_value );
        _pcfg->connect_nodes( return_last_nodes, return_node );
        _utils->_last_nodes.clear( );
        _utils->_return_nodes.append( return_node );
        return ObjectList<Node*>( );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Sizeof& n )
    {
        return visit_unary_node( n, n.get_size_type( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::StringLiteral& n )
    {
        return visit_literal_node( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::StructuredValue& n )
    {
        ObjectList<Node*> items = walk( n.get_items( ) );
        return ObjectList<Node*>( 1, merge_nodes( n, items ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::SwitchStatement& n )
    {
        // Build condition node
        ObjectList<Node*> cond_last_nodes = _utils->_last_nodes;
        ObjectList<Node*> cond_node_l = walk( n.get_switch( ) );
        _pcfg->connect_nodes( cond_last_nodes, cond_node_l[0] );

        // Compose the statements nodes
        _utils->_last_nodes.clear( );
        Node* switch_exit = new Node( );
        _switch_cond_s.push( cond_node_l[0] );
        _utils->_break_nodes.push( switch_exit );
        ObjectList<Node*> case_stmts = walk( n.get_statement( ) );
        _utils->_break_nodes.pop( );

        // Link properly the exit node
        switch_exit->set_id( ++( _utils->_nid ) );
        switch_exit->set_outer_node( _utils->_outer_nodes.top( ) );

        // Finish computation of switch exit nodes
        if( cond_node_l[0]->get_exit_edges( ).empty( ) )
        {   // There is no node node inside the statement
            _pcfg->connect_nodes( cond_node_l[0], switch_exit );
        }
        else
        {   // If there is some node in '_last_nodes' we connect it to the exit (Last Case stmt have not a Break stmt)
            _pcfg->connect_nodes( _utils->_last_nodes, switch_exit );
        }

        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( switch_exit );

        return cond_node_l;
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Symbol& n )
    {
        // Tag the symbol if it is a global variable
        Scope s_sc = n.get_symbol( ).get_scope( );
        Nodecl::NodeclBase n2 = n;
        if( !s_sc.scope_is_enclosed_by( _pcfg->_sc ) )
        {
            Utils::ExtendedSymbolUsage glob_var_usage( n, Utils::undefined_usage );
            if( !Utils::usage_list_contains_sym( glob_var_usage.get_nodecl( ).get_symbol( ), _pcfg->_global_vars ) )
                _pcfg->_global_vars.insert( glob_var_usage );
        }

        // Create the node
        return visit_literal_node( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Throw& n )
    {
        ObjectList<Node*> right = walk( n.get_rhs( ) );
        Node* throw_node;
        if( right.empty( ) )
        {   // Throw has no expression associated, we create now the throw node
            throw_node = _pcfg->append_new_node_to_parent( _utils->_last_nodes, n );
        }
        else
        {   // Some expression has been built. We merge here the node with the whole throw node
            throw_node = merge_nodes( n, right[0], NULL );
            _pcfg->connect_nodes( _utils->_last_nodes, throw_node );
        }

        if( !_utils->_tryblock_nodes.empty( ) )
        {
            for( ObjectList<PCFGTryBlock*>::reverse_iterator it = _utils->_tryblock_nodes.rbegin( );
                it != _utils->_tryblock_nodes.rend( ); ++it )
            {
                ( *it )->_handler_parents.append( throw_node );
            }
        }
        // Throw must be connected to the Graph exit as well
        _pcfg->connect_nodes( throw_node, _pcfg->_graph->get_graph_exit_node( ) );

        _utils->_last_nodes.clear( );
        return ObjectList<Node*>( 1, throw_node );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::TopLevel& n )
    {
        return walk( n.get_top_level( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::TryBlock& n )
    {
        PCFGTryBlock* new_try_block = new PCFGTryBlock( );
        _utils->_tryblock_nodes.append( new_try_block );
        ObjectList<Node*> try_parents = _utils->_last_nodes;
        ObjectList<Node*> try_stmts = walk( n.get_statement( ) );

        Node* first_try_node = try_parents[0]->get_exit_edges( )[0]->get_target( );
        compute_catch_parents( first_try_node );
        _pcfg->clear_visits( first_try_node );

        ObjectList<Node*> handlers_l = walk( n.get_catch_handlers( ) );

        // Process the ellipsis
        ObjectList<Node*> ellipsis_parents = _utils->_last_nodes;
        PCFGTryBlock* current_tryblock = _utils->_tryblock_nodes.back( );
        _utils->_last_nodes = current_tryblock->_handler_parents;
        ObjectList<Node*> ellipsis_l = walk( n.get_any( ) );
        if( !ellipsis_l.empty( ) )
        {
            current_tryblock->_nhandlers++;
            current_tryblock->_handler_exits.append( _utils->_last_nodes );

            // Set the type of the edge between each handler parent and the actual handler
            for( ObjectList<Node*>::iterator it = current_tryblock->_handler_parents.begin( );
                 it != current_tryblock->_handler_parents.end( ); ++it )
            {
                Edge* catch_edge = ( *it )->get_exit_edges( ).back( );
                catch_edge->set_data( _EDGE_TYPE, CATCH_EDGE );
                catch_edge->set_data( _EDGE_LABEL, ObjectList<Nodecl::NodeclBase>( 1, Nodecl::NodeclBase::null( ) ) );
            }
        }

        _utils->_last_nodes = current_tryblock->_handler_exits;

        _utils->_tryblock_nodes.pop_back( );
        if( !try_stmts.empty( ) )
            return try_stmts;
        else if( !handlers_l.empty( ) )
            return handlers_l;
        else if( !ellipsis_l.empty( ) )
            return ellipsis_l;

        return PCFGVisitor::Ret( );
    }

    //TODO Test this kind of node
    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Type& n )
    {
        return visit_literal_node( n );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::Typeid& n )
    {
        return visit_unary_node( n, n.get_arg( ) );
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::VirtualFunctionCall& n )
    {
        return visit_function_call(n);
    }

    PCFGVisitor::Ret PCFGVisitor::visit( const Nodecl::WhileStatement& n )
    {
        // Build condition node
        ObjectList<Node*> cond_last_nodes = _utils->_last_nodes;
        ObjectList<Node*> cond_node_l = walk( n.get_condition( ) );
        Node* cond_node = cond_node_l[0];
        _pcfg->connect_nodes( cond_last_nodes, cond_node );
        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( cond_node );

        Node* exit_node = new Node( );

        // Build the while body node/s
        _utils->_continue_nodes.push( cond_node );
        _utils->_break_nodes.push( exit_node );
        walk( n.get_statement( ) );    // This list of nodes returned here will never be used
        _utils->_continue_nodes.pop( );
        _utils->_break_nodes.pop( );

        ObjectList<Edge*> cond_exits = cond_node->get_exit_edges( );
        Edge_type aux_etype = ALWAYS_EDGE;
        if( !cond_exits.empty( ) )
        {
            // The first edge and, and, if the first is a task, all the following edges being tasks and the first not being a task are TRUE_EDGE
            // If all exit exit edges are tasks, then the "Next node" of the loop is linked with a true edge as well
            ObjectList<Edge*>::iterator it = cond_exits.begin( );
            while( it != cond_exits.end( ) )
            {
                if( !( *it )->is_task_edge( ) )
                {
                    ( *it )->set_data( _EDGE_TYPE, TRUE_EDGE );
                }
                else
                {
                    ( *it )->set_data( _EDGE_TYPE, TRUE_EDGE );
                }
                ++it;
            }
        }
        else
        {
            aux_etype = TRUE_EDGE;
        }
        _pcfg->connect_nodes( cond_node, exit_node, FALSE_EDGE );

        // Build the exit node
        exit_node->set_id( ++( _utils->_nid ) );
        exit_node->set_outer_node( _utils->_outer_nodes.top( ) );
        int n_connects = _utils->_last_nodes.size( );

        _pcfg->connect_nodes( _utils->_last_nodes, cond_node,
                              ObjectList<Edge_type>( n_connects, aux_etype ),
                              ObjectList<std::string>( n_connects, "" ) );

        _utils->_last_nodes.clear( );
        _utils->_last_nodes.append( exit_node );

        return cond_node_l;
    }

    // ******************************** END visiting methods ******************************** //
    // ************************************************************************************** //

}
}
