/*--------------------------------------------------------------------
  (C) Copyright 2015-2015 Barcelona Supercomputing Center
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


#ifndef TL_NANOS6_TASK_PROPERTIES_HPP
#define TL_NANOS6_TASK_PROPERTIES_HPP

#include "tl-nanos6.hpp"
#include "tl-nanos6-directive-environment.hpp"
#include "tl-nanos6-device.hpp"

#include "tl-datareference.hpp"

#include "tl-object.hpp"
#include "tl-nodecl.hpp"
#include "tl-nodecl-utils.hpp"
#include "tl-type.hpp"
#include "tl-symbol.hpp"


namespace TL { namespace Nanos6 {

    // Forward declaration of TL::Nanos6::Lower
    class Lower;

    struct TaskProperties
    {
        private:
            typedef std::map<TL::Symbol, TL::Symbol> field_map_t;
            typedef std::map<TL::Symbol, TL::Symbol> array_descriptor_map_t;

            struct TaskloopBounds
            {
                Nodecl::NodeclBase lower_bound;
                Nodecl::NodeclBase upper_bound;
                Nodecl::NodeclBase step;
            };

            //! This member represents the directive environment
            DirectiveEnvironment _env;
            TL::ObjectList<std::shared_ptr<Device> > _implementations;

            LoweringPhase* _phase;

            //FIXME: Once we have the new implementation of task reductions we may be able to remove this member
            Lower* _lower_visitor;

            field_map_t _field_map;

            array_descriptor_map_t _array_descriptor_map;

            static const int VLA_OVERALLOCATION_ALIGN = 8;

            //FIXME: Once we have the new implementation of task reductions we may be able to remove this member
            unsigned int _num_reductions;

            //! It's used (among other things) to avoid name collision when generating new functions
            int _nanos6_task_counter;

            TL::Type _info_structure;

            TL::Symbol _dependences_function;
            TL::Symbol _dependences_function_mangled;

            TL::Symbol _priority_function;
            TL::Symbol _priority_function_mangled;

            TaskloopBounds _taskloop_bounds;

            Nodecl::NodeclBase _task_body;

            // For inline _related_function is the enclosing task,
            // for function tasks, it is the function task itself
            TL::Symbol _related_function;

            const locus_t* _locus_of_task_creation;
            const locus_t* _locus_of_task_declaration;

        private:

            TL::Symbol create_outline_function(std::shared_ptr<Device> device);

            void create_dependences_function();
            void create_dependences_function_c();

            void create_dependences_function_fortran();
            void create_dependences_function_fortran_proper();
            void create_dependences_function_fortran_forward();
            void create_dependences_function_fortran_mangled();


            TL::Symbol create_constraints_function() const;
            void create_cost_function();
            void create_priority_function();

            TL::Symbol add_field_to_class(TL::Symbol new_class_symbol,
                                          TL::Scope class_scope,
                                          const std::string &var_name,
                                          const locus_t *var_locus,
                                          bool is_allocatable,
                                          TL::Type field_type);

            TL::Scope compute_scope_for_environment_structure();

            Nodecl::NodeclBase rewrite_expression_using_args(
                TL::Symbol args,
                Nodecl::NodeclBase expr,
                const TL::ObjectList<TL::Symbol> &local) const;

            TL::Type rewrite_type_using_args(
                    TL::Symbol arg,
                    TL::Type t,
                    const TL::ObjectList<TL::Symbol> &local) const;

            void compute_reduction_arguments_register_dependence(
                    TL::DataReference& data_ref,
                    // Out
                    TL::ObjectList<Nodecl::NodeclBase>& arguments_list);

            void register_dependence_c(
                    TL::DataReference& data_ref,
                    TL::Symbol handler,
                    TL::Symbol arg,
                    TL::Symbol register_fun,
                    const TL::ObjectList<TL::Symbol>& local_symbols,
                    // Out
                    Nodecl::List& register_statements);

            /** Note that the list of local_symbols may be modified **/
            void register_multidependence_c(
                    TL::DataReference& data_ref,
                    TL::Symbol handler,
                    TL::Symbol arg,
                    TL::Symbol register_fun,
                    TL::Scope scope,
                    // Out
                    TL::ObjectList<TL::Symbol>& local_symbols,
                    Nodecl::List& register_statements);

            void register_multidependence_fortran(
                    TL::DataReference &data_ref,
                    TL::Symbol handler,
                    Nodecl::Utils::SymbolMap &symbol_map,
                    TL::Symbol register_fun,
                    TL::Scope scope,
                    // Out
                    Nodecl::List &register_statements);

            void register_dependence_fortran(
                    TL::DataReference &data_ref,
                    TL::Symbol handler,
                    Nodecl::Utils::SymbolMap &symbol_map,
                    TL::Symbol register_fun,
                    // Out
                    Nodecl::List &register_statements);

            void create_static_variable_depending_on_function_context(
                    const std::string &var_name,
                    TL::Type var_type,
                    // Out
                    TL::Symbol &new_var) const;

            void create_static_variable_regular_function(
                    const std::string &var_name,
                    TL::Type var_type,
                    // Out
                    TL::Symbol &new_var) const;

            void create_static_variable_nondependent_function(
                    const std::string &var_name,
                    TL::Type var_type,
                    // Out
                    TL::Symbol &new_var) const;

            void create_static_variable_dependent_function(
                    const std::string &var_name,
                    TL::Type var_type,
                    // Out
                    TL::Symbol &new_var) const;

            void compute_captured_saved_expressions();

            /* This function traverses all the expressions that are evaluated using the arguments structure
             * (e.g. dependences, cost, priority) and firstprivatize the symbols that don't have a data-sharing
             *
             * It may add symbols that represent saved_expressions to the captured_values list.
             */
            void firstprivatize_symbols_without_data_sharing();

        public:
            TaskProperties(
                    const Nodecl::OpenMP::Task& node,
                    LoweringPhase* lowering_phase,
                    Lower* lower);

            // FIXME: This constructor shouldn't exist
            TaskProperties(const Nodecl::OmpSs::Release& node, LoweringPhase* lowering_phase, Lower* lower);

            // FIXME
            std::string get_new_name(const std::string& prefix) const;

            //! This function creates a new static variable that contains the task invocation information
            void create_task_invocation_info(
                /* out */ TL::Symbol &task_invocation_info);

            //! This function creates a new global static array variable that contains all
            //! the information related to each possible implementation of the current task
            void create_task_implementations_info(
                    /* out */
                    TL::Symbol &implementations);

            //! This function creates a new global static variable that contains all
            //! the information associated with a task
            void create_task_info(
                    TL::Symbol implementations,
                    /* out */
                    TL::Symbol &task_info);

            //! This function creates a new class type that represents the arguments structure.
            /*!
             * @param data_env_struct The new class type
             * @param arg_size An expression that evaluates to the number of bytes to be allocated
             * @param requires_initialization This boolean states whether the current argument structure should be initialized
             */
            void create_environment_structure(
                    /* out */
                    TL::Type& data_env_struct,
                    Nodecl::NodeclBase& args_size,
                    bool &requires_initialization);

            void capture_environment(
                    TL::Symbol args,
                    TL::Scope task_enclosing_scope,
                    /* out */
                    Nodecl::NodeclBase& capture_env);

            void compute_task_flags(
                    TL::Symbol task_flags,
                    /* out */
                    Nodecl::NodeclBase& task_flags_stmts);

            void handle_task_reductions(
                    const TL::Scope& unpacked_inside_scope,
                    Nodecl::NodeclBase unpacked_empty_stmt);

            void compute_release_statements(/* out */ Nodecl::List& release_stmts);

            void fortran_add_types(TL::Scope sc);

            bool symbol_has_data_sharing_attribute(TL::Symbol sym) const;

            static bool is_saved_expression(Nodecl::NodeclBase n);

            bool task_is_loop() const;

            Nodecl::NodeclBase get_lower_bound() const;
            Nodecl::NodeclBase get_upper_bound() const;
            Nodecl::NodeclBase get_step() const;
            Nodecl::NodeclBase get_chunksize() const;
    };

} }

#endif // TL_NANOS6_TASK_PROPERTIES_HPP
