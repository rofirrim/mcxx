/*--------------------------------------------------------------------
  (C) Copyright 2017-2017 Barcelona Supercomputing Center
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


#include "codegen-fortran-llvm.hpp"

namespace Codegen { 

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

// Based on ioparm.def from gfortran
#define IOPARM_LIST_FIELDS \
 IOPARM_START(common) \
  IOPARM (common,  flags,		0,	 int4) \
  IOPARM (common,  unit,		0,	 int4) \
  IOPARM (common,  filename,	0,	 pchar) \
  IOPARM (common,  line,		0,	 int4) \
  IOPARM (common,  iomsg,		1 << 6,  char2) \
  IOPARM (common,  iostat,	1 << 5,  pint4) \
 IOPARM_END(common) \
 IOPARM_START(dt) \
  IOPARM (dt,      common,	0,	 common) \
  IOPARM (dt,      rec,		1 << 9,  intio) \
  IOPARM (dt,      size,		1 << 10, pintio) \
  IOPARM (dt,      iolength,	1 << 11, pintio) \
  IOPARM (dt,      internal_unit_desc, 0,  parray) \
  IOPARM (dt,      format,	1 << 12, char1) \
  IOPARM (dt,      advance,	1 << 13, char2) \
  IOPARM (dt,      internal_unit,	1 << 14, char1) \
  IOPARM (dt,      namelist_name,	1 << 15, char2) \
  IOPARM (dt,      u,		0,	 pad) \
  IOPARM (dt,      id,		1 << 16, pint4) \
  IOPARM (dt,      pos,		1 << 17, intio) \
  IOPARM (dt,      asynchronous, 	1 << 18, char1) \
  IOPARM (dt,      blank,		1 << 19, char2) \
  IOPARM (dt,      decimal,	1 << 20, char1) \
  IOPARM (dt,      delim,		1 << 21, char2) \
  IOPARM (dt,      pad,		1 << 22, char1) \
  IOPARM (dt,      round,		1 << 23, char2) \
  IOPARM (dt,      sign,		1 << 24, char1) \
 IOPARM_END(dt)

void FortranLLVM::initialize_gfortran_runtime()
{
    typedef std::vector<llvm::Type *> TL;
    auto add_type_int4 = [=](TL &t) { t.push_back(llvm_types.i32); };
    auto add_type_intio = [=](TL &t) { add_type_int4(t); };
    auto add_type_pint4 = [=](TL &t) { t.push_back(llvm_types.ptr_i32); };
    auto add_type_pchar = [=](TL &t) { t.push_back(llvm_types.ptr_i8); };
    auto add_type_char1 = [=](TL &t) {
        t.push_back(llvm_types.i32);
        add_type_pchar(t);
    };
    auto add_type_char2 = [=](TL &t) {
        add_type_pchar(t);
        t.push_back(llvm_types.i32);
    };
    auto add_type_pintio = [=](TL &t) { t.push_back(llvm_types.i64); };
    auto add_type_parray = [=](TL &t) { add_type_pchar(t); };
    auto add_type_pad = [=](TL &t) {
        const llvm::DataLayout &dl = current_module->getDataLayout();
        uint64_t size = 16 * dl.getPointerTypeSize(llvm_types.ptr_i8)
                        + 32 * dl.getTypeAllocSize(llvm_types.i32);
        t.push_back(llvm::ArrayType::get(llvm_types.i8, size));
    };
    auto add_type_common
        = [=](TL &t) { t.push_back(gfortran_rt.st_parameter_common.get()); };

#define IOPARM_START(name) auto create_##name = [=]() { \
         llvm::StructType *ioparm_type_##name = llvm::StructType::create( \
             llvm_context, "st_parameter_" #name); \
         std::vector<llvm::Type *> name##_elements;
#define IOPARM(name, field_name, __, type) \
    add_type_##type(name##_elements);      \
    fields[ioparm_type_##name].add_field(#field_name);
#define IOPARM_END(name)                                  \
    ioparm_type_##name->setBody(name##_elements);         \
    return static_cast<llvm::Type *>(ioparm_type_##name); \
    }                                                     \
    ;
    IOPARM_LIST_FIELDS
#undef IOPARM_END
#undef IOPARM
#undef IOPARM_START

    this->gfortran_rt.st_parameter_common = create_common;
    this->gfortran_rt.st_parameter_dt = create_dt;

    this->gfortran_rt.st_write = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(
                llvm_types.void_,
                { gfortran_rt.st_parameter_dt.get()->getPointerTo() },
                /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_st_write",
            current_module.get());
    };

    this->gfortran_rt.transfer_character_write = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(
                llvm_types.void_,
                { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                  llvm_types.ptr_i8,
                  llvm_types.i32 },
                /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_transfer_character_write",
            current_module.get());
    };

    this->gfortran_rt.transfer_integer_write = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(
                llvm_types.void_,
                { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                  llvm_types.ptr_i8,
                  llvm_types.i32 },
                /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_transfer_integer_write",
            current_module.get());
    };

    this->gfortran_rt.transfer_real_write = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(
                llvm_types.void_,
                { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                  llvm_types.ptr_i8,
                  llvm_types.i32 },
                /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_transfer_real_write",
            current_module.get());
    };

    this->gfortran_rt.transfer_complex_write = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(
                llvm_types.void_,
                { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                  llvm_types.ptr_i8,
                  llvm_types.i32 },
                /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_transfer_complex_write",
            current_module.get());
    };

    this->gfortran_rt.transfer_logical_write = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(
                llvm_types.void_,
                { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                  llvm_types.ptr_i8,
                  llvm_types.i32 },
                /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_transfer_logical_write",
            current_module.get());
    };

    this->gfortran_rt.transfer_array_write = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(
                llvm_types.void_,
                { gfortran_rt.st_parameter_dt.get()->getPointerTo(),
                  llvm_types.ptr_i8,
                  llvm_types.i32,
                  llvm_types.i32 },
                /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_transfer_array_write",
            current_module.get());
    };

    this->gfortran_rt.st_write_done = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(
                llvm_types.void_,
                { gfortran_rt.st_parameter_dt.get()->getPointerTo() },
                /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_st_write_done",
            current_module.get());
    };

    this->gfortran_rt.set_args = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(
                llvm_types.void_,
                { llvm_types.i32, llvm_types.ptr_i8->getPointerTo() },
                /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_set_args",
            current_module.get());
    };

    this->gfortran_rt.set_options = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(llvm_types.void_,
                                    { llvm_types.i32, llvm_types.ptr_i32 },
                                    /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_set_options",
            current_module.get());
    };

    this->gfortran_rt.stop_int = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(llvm_types.void_,
                                    { llvm_types.i32 },
                                    /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_stop_numeric_f08",
            current_module.get());
    };

    this->gfortran_rt.descriptor_dimension = [&, this]() -> llvm::Type * {
        // struct descriptor_dimension
        // {
        //   ptrdiff_t stride;
        //   ptrdiff_t lower_bound;
        //   ptrdiff_t upper_bound;
        // };
        // FIXME - We need the right ptrdiff_t here
        llvm::Type *pdiff = llvm_types.i64;
        llvm::StructType *t
            = llvm::StructType::create(llvm_context, "descriptor_dimension");
        fields[t].add_field("stride");
        fields[t].add_field("lower_bound");
        fields[t].add_field("upper_bound");
        t->setBody({ pdiff, pdiff, pdiff });
        return t;
    };

    this->gfortran_rt.malloc = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(llvm_types.ptr_i8,
                                    // FIXME - Make this a size_t
                                    { llvm_types.i64 },
                                    /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "malloc",
            current_module.get());
    };

    this->gfortran_rt.free = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(llvm_types.void_,
                                    { llvm_types.ptr_i8 },
                                    /* isVarArg */ false),
            llvm::GlobalValue::ExternalLinkage,
            "free",
            current_module.get());
    };

    this->gfortran_rt.runtime_error_at = [&, this]() {
        return llvm::Function::Create(
            llvm::FunctionType::get(llvm_types.void_,
                                    { llvm_types.ptr_i8, llvm_types.ptr_i8 },
                                    /* isVarArg */ true),
            llvm::GlobalValue::ExternalLinkage,
            "_gfortran_runtime_error_at",
            current_module.get());
    };
}

llvm::Type *FortranLLVM::get_gfortran_array_descriptor_type(TL::Type t)
{
    ERROR_CONDITION(!t.is_fortran_array(), "Invalid type", 0);

    int rank = t.fortran_rank();

    auto it = gfortran_rt.array_descriptor.find(rank);
    if (it != gfortran_rt.array_descriptor.end())
        return it->second;

    std::stringstream ss;
    ss << "descriptor_rank_" << rank;
    llvm::StructType *struct_type
        = llvm::StructType::create(llvm_context, ss.str());

    // template <int Rank>
    // struct descriptor
    // {
    //     void *base_addr;
    //     size_t offset;
    //     ptrdiff_t dtype;
    //     descriptor_dimension dim[Rank];
    // };

    std::vector<llvm::Type *> field_types;
    field_types.reserve(4);

    fields[struct_type].add_field("base_addr");
    field_types.push_back(llvm_types.ptr_i8);

    fields[struct_type].add_field("offset");
    // Fixme we need a sensible size_t here
    field_types.push_back(llvm_types.i64);

    fields[struct_type].add_field("dtype");
    // Fixme we need a sensible ptrdiff_t here
    field_types.push_back(llvm_types.i64);

    fields[struct_type].add_field("dim");
    field_types.push_back(
        llvm::ArrayType::get(gfortran_rt.descriptor_dimension.get(), rank));

    struct_type->setBody(field_types);

    gfortran_rt.array_descriptor.insert(std::make_pair(rank, struct_type));

    return struct_type;
}

void FortranLLVM::emit_main(llvm::Function *fortran_program)
{
    // FIXME - This is hardcoded
    // FIXME - Move this to initialize_gfortran_runtime
    // static integer(kind=4) options.1[9] = {68, 1023, 0, 0, 1, 1, 0, 0, 31};
    llvm::ArrayType *options_type = llvm::ArrayType::get(llvm_types.i32, 9);
    std::vector<llvm::Constant *> options_values{
        get_integer_value_32(68), get_integer_value_32(1023),
        get_integer_value_32(0),  get_integer_value_32(0),
        get_integer_value_32(1),  get_integer_value_32(1),
        get_integer_value_32(0),  get_integer_value_32(0),
        get_integer_value_32(31)
    };

    llvm::GlobalVariable *options = new llvm::GlobalVariable(
        *current_module,
        options_type,
        /* isConstant */ true,
        llvm::GlobalValue::PrivateLinkage,
        llvm::ConstantArray::get(options_type, options_values));

    llvm::AttributeList attributes;
    attributes = attributes.addAttribute(llvm_context,
                                         llvm::AttributeList::FunctionIndex,
                                         llvm::Attribute::NoRecurse);
    attributes = attributes.addAttribute(llvm_context,
                                         llvm::AttributeList::FunctionIndex,
                                         llvm::Attribute::UWTable);
    attributes = attributes.addAttribute(llvm_context,
                                         llvm::AttributeList::FunctionIndex,
                                         llvm::Attribute::NoUnwind);
    llvm::Constant *c = current_module->getOrInsertFunction(
        "main",
        llvm::FunctionType::get(
            llvm_types.i32,
            { llvm_types.i32, llvm_types.ptr_i8->getPointerTo() },
            /* isVarArg */ false),
        attributes);

    llvm::Function *fun = llvm::cast<llvm::Function>(c);

    set_current_function(fun);
    llvm::BasicBlock *entry_basic_block
        = llvm::BasicBlock::Create(llvm_context, "entry", fun);
    set_current_block(entry_basic_block);

    std::vector<llvm::Value *> main_args;
    for (llvm::Argument &v : fun->args())
    {
        main_args.push_back(&v);
    }
    ir_builder->CreateCall(gfortran_rt.set_args.get(), main_args);

    ir_builder->CreateCall(
        gfortran_rt.set_options.get(),
        { get_integer_value_32(9),
          ir_builder->CreatePointerCast(options, llvm_types.ptr_i32) });

    ir_builder->CreateCall(fortran_program, {});

    ir_builder->CreateRet(get_integer_value_32(0));

    clear_current_function();
}

}
