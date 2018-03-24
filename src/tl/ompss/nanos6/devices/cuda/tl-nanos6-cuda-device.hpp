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

#ifndef TL_NANOS6_CUDA_DEVICE_HPP
#define TL_NANOS6_CUDA_DEVICE_HPP

#include "tl-nanos6-device.hpp"

namespace TL { namespace Nanos6 {

    class CUDADevice : public Device
    {
        private:
            Nodecl::List _cuda_code;

        public:
            CUDADevice();

            ~CUDADevice();

            //! This function returns a symbol that represents the device type id
            TL::Symbol get_device_type_id() const;

            //! It generates a CUDA kernel call if the ndrange clause was present.
            //! Otherwise it returns a copy of the task_body
            Nodecl::NodeclBase compute_specific_task_body(
                    Nodecl::NodeclBase task_body, const DirectiveEnvironment &env) const;

            void root_unpacked_function(TL::Symbol unpacked_function, Nodecl::NodeclBase unpacked_function_code);



        private:

            void compile_cuda_code() const;
    };

} }

#endif // TL_NANOS6_CUDA_DEVICE_HPP
