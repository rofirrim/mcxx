/*--------------------------------------------------------------------
  
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

#include "tl-compilerphase.hpp"
#include "codegen-fortran-llvm.hpp"
#include "fortran03-buildscope.h"
#include "fortran03-scope.h"
#include "fortran03-exprtype.h"
#include "fortran03-typeutils.h"
#include "fortran03-cexpr.h"
#include "tl-compilerpipeline.hpp"
#include "tl-source.hpp"
#include "cxx-cexpr.h"
#include "cxx-entrylist.h"
#include "cxx-driver-utils.h"
#include "cxx-diagnostic.h"
#include "string_utils.h"
#include <ctype.h>

#include "cxx-lexer.h"

#include <llvm/Support/raw_os_ostream.h>

namespace Codegen
{
    FortranLLVM::FortranLLVM()
    {
        set_phase_name("Fortran LLVM codegen");
        set_phase_description("This phase emits in LLVM IR the intermediate representation of the compiler");
    }

    void FortranLLVM::codegen(const Nodecl::NodeclBase &n, std::ostream* out)
    {
        if (!is_file_output())
        {
            // Let FortranBase handle this case
            base.codegen(n, out);
        }
        else
        {
            file = out;
            walk(n);
        }
    }

    void FortranLLVM::codegen_cleanup()
    {
    }

    void FortranLLVM::visit(const Nodecl::TopLevel& node)
    {
        std::unique_ptr<llvm::Module> old_module = std::move(current_module);
        // FIXME: Name??
        current_module = llvm::make_unique<llvm::Module>(
            TL::CompilationProcess::get_current_file().get_filename(),
            llvm_context);

        walk(node.get_top_level());

        // current_module->dump();
        llvm::raw_os_ostream ros(*file);
        current_module->print(ros,
                /* AssemblyAnnotationWriter */ nullptr);

        std::swap(old_module, current_module);
    }

    void FortranLLVM::visit(const Nodecl::FunctionCode& node)
    {
        // Create a function with the proper type
    }
}


EXPORT_PHASE(Codegen::FortranLLVM)
