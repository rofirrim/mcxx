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

void FortranLLVM::emit_variable(TL::Symbol sym)
{
    ERROR_CONDITION(!sym.is_variable(),
                    "Invalid symbol kind '%s'\n",
                    symbol_kind_name(sym.get_internal_symbol()));
    if (get_value(sym) != NULL)
        return;

    llvm::Value *array_size = nullptr;
    if (sym.get_type().is_fortran_array()
        && !sym.get_type().array_requires_descriptor())
    {
        // Emit size
        TrackLocation loc(this, sym.get_locus());

        TL::Type t = sym.get_type();
        array_size = eval_size_of_array(t);
    }

    llvm::Type *llvm_type = get_llvm_type(sym.get_type());
    llvm::Value *allocation
        = ir_builder->CreateAlloca(llvm_type, array_size, sym.get_name());
    map_symbol_to_value(sym, allocation);

    // We set FlagArtificial to avoid a bug in the verifier if DILocalVariable
    llvm::DINode::DIFlags flags = llvm::DINode::FlagArtificial;

    llvm::DILocalVariable *dbg_var
        = dbg_builder->createAutoVariable(get_debug_scope(),
                                          sym.get_name(),
                                          dbg_info.file,
                                          sym.get_line(),
                                          get_debug_info_type(sym.get_type()),
                                          /* AlwaysPreserve */ false,
                                          flags);

    std::vector<int64_t> dbg_expr_ops;
    llvm::DIExpression *dbg_expr = dbg_builder->createExpression(dbg_expr_ops);

    dbg_builder->insertDeclare(
        allocation,
        dbg_var,
        dbg_expr,
        llvm::DILocation::get(
            llvm_context, sym.get_line(), sym.get_column(), get_debug_scope()),
        ir_builder->GetInsertBlock());

    // If this variable is ALLOCATABLE or a POINTER it must be set to zero
    if (sym.is_allocatable() || sym.get_type().is_pointer())
    {
        if ((sym.get_type().is_array()
             && sym.get_type().array_requires_descriptor())
            || (sym.get_type().is_pointer()
                && sym.get_type().points_to().is_array()))
        {
            FortranLLVM::TrackLocation loc(this, sym.get_locus());
            // This has created a descriptor. Set it to zero.
            // FIXME - I think there is a zeroinitializer for these cases.
            ir_builder->CreateStore(
                llvm::ConstantPointerNull::get(llvm_types.ptr_i8),
                array_descriptor_addr_base_addr(allocation));
        }
        else
        {
            internal_error("Not implemented yet", 0);
        }
    }
}

void FortranLLVM::gfortran_runtime_error(const locus_t *locus,
                                         const std::string &str)
{
    std::stringstream ss;
    ss << "At " << locus_to_str(locus);

    ir_builder->CreateCall(gfortran_rt.runtime_error_at.get(),
                           { ir_builder->CreateGlobalStringPtr(ss.str()),
                             ir_builder->CreateGlobalStringPtr("%s"),
                             ir_builder->CreateGlobalStringPtr(str) });
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
