/*--------------------------------------------------------------------
  (C) Copyright 2006-2014 Barcelona Supercomputing Center
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

#ifndef TL_VECTORIZER_OVERLAP_HPP
#define TL_VECTORIZER_OVERLAP_HPP


#include <map>
#include <vector>

#include "tl-nodecl.hpp"
#include "tl-nodecl-visitor.hpp"
#include "tl-vectorizer-overlap-fwd.hpp"
#include "tl-vectorizer-environment.hpp"
#include "tl-vectorization-analysis-interface.hpp"


namespace TL
{
    namespace Vectorization
    {
        /*
        class OverlapInfo
        {
            private:
                Nodecl::NodeclBase _lower_bound;
                Nodecl::NodeclBase _upper_bound;
                Nodecl::NodeclBase _stride;
                const int _overlap_factor; 

                std::vector<TL::Symbol> _register_list;

            public:
                OverlapInfo(const Nodecl::NodeclBase& lower_bound,
                       const Nodecl::NodeclBase& upper_bound,
                       const Nodecl::NodeclBase& stride,
                       const int overlap_factor);

            friend class VectorizerOverlap;

        };

        class VectorizerOverlap
        {
            private:
                typedef std::map<TL::Symbol, OverlapInfo> overlap_map_t; //Second will be a list
                typedef std::pair<TL::Symbol, OverlapInfo> overlap_pair_t; //Second will be a list

                overlap_map_t _overlap_map;

            public:
                VectorizerOverlap(
                        const tl_sym_int_map_t& overlapd_expressions);

                void declare_overlap_symbols(TL::Scope scope,
                        const VectorizerEnvironment& environment);

                Nodecl::List get_init_statements(VectorizerEnvironment& environment) const;
                Nodecl::List get_iteration_update_pre(VectorizerEnvironment& environment) const;
                Nodecl::List get_iteration_update_post(VectorizerEnvironment& environment) const;

                bool is_overlapd_access(const Nodecl::ArraySubscript& n) const;
                Nodecl::NodeclBase get_load_access(const Nodecl::ArraySubscript& n) const;
        };
*/
        struct OverlapGroup
        {
            Nodecl::Symbol _subscripted;
            objlist_tlsym_t _registers;
            objlist_nodecl_t _registers_indexes;
            objlist_nodecl_t _loads;
            Nodecl::VectorLoad _leftmost_vload;
            Nodecl::VectorLoad _rightmost_vload;
            Nodecl::NodeclBase _loop_ind_var;
            Nodecl::NodeclBase _loop_ind_var_step;
            TL::Type _basic_type;
            TL::Type _vector_type;
            int _num_registers;
            bool _aligned_strategy;
            bool _inter_it_overlap;

            bool overlaps(const Nodecl::VectorLoad& vector_load);
            Nodecl::List get_init_statements(
                    const Nodecl::ForStatement& for_stmt,
                    const bool is_simd_loop,
                    const bool is_omp_simd_for,
                    const bool inter_iteration_overlap) const;
            Nodecl::List get_iteration_update_pre() const;
            Nodecl::List get_iteration_update_post() const;

            void compute_leftmost_rightmost_vloads(
                    const Vectorization::VectorizerEnvironment& environment,
                    const int max_registers);
            void leftmost_rightmost_strategy(
                    const Vectorization::VectorizerEnvironment& environment,
                    const bool aligned_strategy);
 
            void compute_basic_properties();
            void compute_inter_iteration_overlap();

            void compute_num_registers(
                    const Vectorization::VectorizerEnvironment& environment);
        };

        typedef TL::ObjectList<OverlapGroup> objlist_ogroup_t;
        class OverlappedAccessesOptimizer : 
            public Nodecl::ExhaustiveVisitor<void>
        {
            private:
                const VectorizerEnvironment& _environment;
                bool _is_omp_simd_for;
                bool _is_simd_epilog;
                Nodecl::List& _prependix_stmts;
                
                VectorizationAnalysisInterface* _first_analysis;

                static VectorizationAnalysisInterface* _analysis;

                void update_alignment_info(
                        const Nodecl::NodeclBase& main_loop,
                        const Nodecl::NodeclBase& epilog_loop);

                objlist_nodecl_t get_adjacent_vector_loads_not_nested_in_for(
                        const Nodecl::NodeclBase& n,
                        const TL::Symbol& sym);
                objlist_ogroup_t get_overlap_groups(
                        const objlist_nodecl_t& adjacent_accesses,
                        const int min_group_loads,
                        const int max_group_registers,
                        const int max_groups,
                        const Nodecl::NodeclBase& loop_ind_var,
                        const Nodecl::NodeclBase& loop_ind_var_step);

                void retrieve_group_registers(
                        OverlapGroup& ogroup,
                        TL::Scope& scope,
                        const int num_group);

                bool need_init_cache(
                        const bool is_nested_loop,
                        const bool is_simd_epilog,
                        const bool is_overlap_epilog);
                bool need_update_post(
                        const bool is_nested_loop,
                        const bool is_simd_epilog,
                        const bool is_overlap_epilog);

                void insert_group_update_stmts(
                        OverlapGroup& ogroup,
                        const Nodecl::ForStatement& n,
                        const bool is_overlap_epilog);
                void replace_overlapped_loads(
                        const OverlapGroup& ogroup);

                unsigned int get_loop_min_unroll_factor(
                        Nodecl::ForStatement n);
                Nodecl::ForStatement get_overlap_blocked_unrolled_loop(
                        const Nodecl::ForStatement& n,
                        const unsigned int block_size);

            public:
                OverlappedAccessesOptimizer(VectorizerEnvironment& environment,
                        VectorizationAnalysisInterface* analysis,
                        const bool is_omp_simd_for,
                        const bool is_epilog,
                        Nodecl::List& prependix_stmts);
                
                void visit(const Nodecl::ForStatement&);

            friend struct OverlapGroup;

       };
    }
}

#endif // TL_VECTORIZER_OVERLAP_HPP
