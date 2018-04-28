/*--------------------------------------------------------------------
  (C) Copyright 2006-2013 Barcelona Supercomputing Center
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

#ifndef CODEGEN_PHASE_HPP
#define CODEGEN_PHASE_HPP

#include "tl-compilerphase.hpp"
#include "codegen-common.hpp"

namespace Codegen
{
    class CodegenPhase : public TL::CompilerPhase, public CodegenVisitor
    {
        virtual void run(TL::DTO& dto);

        public:
            virtual void handle_parameter(int n, void* data);
    };

    CodegenPhase& get_current();
}

#define EXPORT_CODEGEN_PHASE(ClassName, CodegenId) \
extern "C"  \
{ \
    TL::CompilerPhase* get_codegen_phase_##CodegenId(void) \
    { \
        return new ClassName(); \
    } \
}

#define DECLARE_CODEGEN_PHASE(CodegenId) \
extern "C" { \
   TL::CompilerPhase* get_codegen_phase_##CodegenId(void); \
}

// FIXME: Make this more scalable
DECLARE_CODEGEN_PHASE(cxx);
DECLARE_CODEGEN_PHASE(fortran);
DECLARE_CODEGEN_PHASE(cuda);

#endif // CODEGEN_PHASE_HPP
