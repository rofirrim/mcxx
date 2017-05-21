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

#include "config.h"
#include "codegen-fortran-llvm.hpp"
#include "filename.h"
#include "cxx-driver-build-info.h"
#include "tl-compilerpipeline.hpp"
#include "llvm/Support/raw_os_ostream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"

namespace Codegen
{

FortranLLVM::FortranLLVM()
{
    set_phase_name("Fortran LLVM codegen");
    set_phase_description(
        "This phase emits in LLVM IR the intermediate representation of the "
        "compiler");
}

void FortranLLVM::codegen(const Nodecl::NodeclBase &n, std::ostream *out)
{
    ERROR_CONDITION(
        !is_file_output(), "This codegen is only for file output", 0);

    file = out;
    walk(n);
}

void FortranLLVM::codegen_cleanup()
{
}

namespace
{
void ensure_llvm_initialized()
{
    static bool init = false;
    if (!init)
    {
        llvm::InitializeAllTargetInfos();
        llvm::InitializeAllTargets();
        llvm::InitializeAllTargetMCs();
        init = true;
    }
}
}

void FortranLLVM::visit(const Nodecl::TopLevel &node)
{
    ensure_llvm_initialized();

    std::unique_ptr<llvm::Module> old_module;
    std::swap(old_module, current_module);

    current_module = llvm::make_unique<llvm::Module>(
        TL::CompilationProcess::get_current_file().get_filename(),
        llvm_context);

    current_module->addModuleFlag(llvm::Module::Error, "PIC Level", 2);
    current_module->addModuleFlag(llvm::Module::Warning,
                                  "Debug Info Version",
                                  llvm::DEBUG_METADATA_VERSION);
    current_module->addModuleFlag(llvm::Module::Warning, "Dwarf Version", 4);

    ERROR_CONDITION(CURRENT_CONFIGURATION->type_environment->triplet == NULL,
                    "Invalid triple",
                    0);
    std::string default_triple
        = CURRENT_CONFIGURATION->type_environment->triplet;
    std::string error_message;
    const llvm::Target *target
        = llvm::TargetRegistry::lookupTarget(default_triple, error_message);
    if (target == NULL)
        fatal_error("Cannot get a LLVM target for triple '%s' due to '%s'\n",
                    default_triple.c_str(),
                    error_message.c_str());

    // Generic CPU, no special features or options
    llvm::TargetOptions target_options;
    llvm::Optional<llvm::Reloc::Model> reloc_model;
    llvm::TargetMachine *target_machine = target->createTargetMachine(
        default_triple, "generic", "", target_options, reloc_model);

    // Set triple and data layout of the target machine
    current_module->setTargetTriple(default_triple);
    current_module->setDataLayout(target_machine->createDataLayout());

    ir_builder = std::unique_ptr<llvm::IRBuilder<> >(
        new llvm::IRBuilder<>(llvm_context));
    md_builder
        = std::unique_ptr<llvm::MDBuilder>(new llvm::MDBuilder(llvm_context));
    dbg_builder = std::unique_ptr<llvm::DIBuilder>(
        new llvm::DIBuilder(*current_module));

    std::string base = give_basename(node.get_filename().c_str());
    std::string dir = give_dirname(node.get_filename().c_str());
    dbg_info.reset();
    dbg_info.file = dbg_builder->createFile(base, dir);
    /* llvm::DICompileUnit *dbg_compile_unit = */ dbg_builder
        ->createCompileUnit(llvm::dwarf::DW_LANG_Fortran95,
                            dbg_info.file,
                            PACKAGE " " VERSION " (" MCXX_BUILD_VERSION ")",
                            /* isOptimized */ false,
                            /* Flags */ "",
                            /* RuntimeVersion */ 0);

    initialize_llvm_context();

    push_debug_scope(dbg_info.file);
    walk(node.get_top_level());
    pop_debug_scope();
    ERROR_CONDITION(!dbg_info.stack_debug_scope.empty(),
                    "Stack of debug scopes is not empty",
                    0);

    dbg_builder->finalize();

    llvm::raw_os_ostream ros(*file);
    current_module->print(ros,
                          /* AssemblyAnnotationWriter */ nullptr);

    std::swap(old_module, current_module);
}



void FortranLLVM::initialize_aliasing_info()
{
    // tbaa_root = md_builder->createTBAARoot("Fortran 95 aliasing");
}

void FortranLLVM::initialize_llvm_context()
{
    initialize_llvm_types();
    initialize_gfortran_runtime();
    initialize_aliasing_info();
}
}


EXPORT_PHASE(Codegen::FortranLLVM)
