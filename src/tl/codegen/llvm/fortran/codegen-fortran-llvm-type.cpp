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
#include "cxx-cexpr.h"

namespace Codegen {

llvm::Type *FortranLLVM::get_llvm_type(TL::Type t)
{
    if (t.is_lvalue_reference())
    {
        return llvm::PointerType::get(get_llvm_type(t.references_to()),
                                      /* AddressSpace */ 0);
    }
    // else if (t.is_pointer())
    // {
    //     return PointerType::get(
    //             get_llvm_type(t.points_to()),
    //             /* AddressSpace */ 0);
    // }
    else if (t.is_function())
    {
        TL::ObjectList<TL::Type> params = t.parameters();
        std::vector<llvm::Type *> llvm_params;
        llvm_params.reserve(params.size());

        if (t.lacks_prototype())
        {
            for (TL::Type t : params)
            {
                llvm_params.push_back(llvm_types.ptr_i8);
            }
        }
        else
        {
            for (TL::Type t : params)
            {
                llvm_params.push_back(get_llvm_type(t));
            }
        }
        return llvm::FunctionType::get(get_llvm_type(t.returns()),
                llvm_params,
                /* isVarArg */ false);
    }
    else if (t.is_signed_integral() || t.is_bool())
    {
        return llvm::IntegerType::get(llvm_context, t.get_size() * 8);
    }
    else if (t.is_float())
    {
        return llvm_types.f32;
    }
    else if (t.is_double())
    {
        return llvm_types.f64;
    }
    else if (t.is_char())
    {
        return llvm_types.i8;
    }
    else if (t.is_void())
    {
        return llvm_types.void_;
    }
    else if (t.is_complex())
    {
        llvm::Type *base_type = get_llvm_type(t.complex_get_base_type());
        // tuple: real, imag
        return llvm::VectorType::get(base_type, 2);
    }
    else if (t.is_fortran_character())
    {
        // FIXME - CHARACTER(LEN=*)
        Nodecl::NodeclBase size = t.array_get_size();
        ERROR_CONDITION(!size.is_constant(), "Invalid size", 0);

        return llvm::ArrayType::get(get_llvm_type(t.array_element()),
                                    const_value_cast_to_8(size.get_constant()));
    }
    else if (t.is_fortran_array()
            && !t.array_requires_descriptor())
    {
        // Use the base element type
        return get_llvm_type(t.array_base_element());
    }
    else if (t.is_fortran_array()
            && t.array_requires_descriptor())
    {
        return get_gfortran_array_descriptor_type(t);
    }
    else
    {
        internal_error("Cannot synthesize LLVM type for type '%s'",
                       print_declarator(t.get_internal_type()));
    }
}

llvm::DIType *FortranLLVM::get_debug_info_type(TL::Type t)
{
    if (t.is_lvalue_reference())
    {
        return dbg_builder->createReferenceType(llvm::dwarf::DW_TAG_reference_type,
                get_debug_info_type(t.references_to()),
                /* FIXME */ 8 * TL::Type::get_void_type().get_pointer_to().get_size());
    }
    // else if (t.is_pointer())
    // {
    //     return PointerType::get(
    //             get_debug_info_type(t.points_to()),
    //             /* AddressSpace */ 0);
    // }
    else if (t.is_function())
    {
        TL::ObjectList<TL::Type> params = t.parameters();
        std::vector<llvm::Metadata *> dbg_param_types;
        dbg_param_types.reserve(params.size() + 1);

        dbg_param_types.push_back(get_debug_info_type(t.returns()));
        for (TL::Type t : params)
        {
            dbg_param_types.push_back(get_debug_info_type(t));
        }
        return dbg_builder->createSubroutineType(dbg_builder->getOrCreateTypeArray(dbg_param_types));
    }
    else if (t.is_bool())
    {
        std::stringstream ss;
        ss << "LOGICAL(KIND=" << t.get_size() << ")";
        return dbg_builder->createBasicType(ss.str(), t.get_size() * 8, llvm::dwarf::DW_ATE_boolean);
    }
    else if (t.is_signed_integral())
    {
        std::stringstream ss;
        ss << "INTEGER(KIND=" << t.get_size() << ")";
        return dbg_builder->createBasicType(ss.str(), t.get_size() * 8, llvm::dwarf::DW_ATE_signed);
    }
    else if (t.is_float())
    {
        return dbg_builder->createBasicType("REAL(KIND=4)", 32, llvm::dwarf::DW_ATE_float);
    }
    else if (t.is_double())
    {
        return dbg_builder->createBasicType("REAL(KIND=8)", 64, llvm::dwarf::DW_ATE_float);
    }
    else if (t.is_char())
    {   
        // Should we honor platform dependent signedness?
        return dbg_builder->createBasicType("CHARACTER", 8, llvm::dwarf::DW_ATE_signed_char);
    }
    else if (t.is_void())
    {
        return dbg_builder->createUnspecifiedType("void");
    }
    else if (t.is_complex())
    {
        std::stringstream ss;
        ss << "COMPLEX(KIND=" << t.complex_get_base_type().get_size() << ")";
        std::string complex_typename = ss.str();

        return dbg_builder->createBasicType(complex_typename,
                t.get_size() * 8,
                llvm::dwarf::DW_ATE_complex_float);
    }
    else if ((t.is_fortran_array() || t.is_fortran_character())
            && !t.array_requires_descriptor())
    {
        // Statically sized arrays or VLAs
        // Unfortunately VLAs have to be represented in a very useless way
        // since DIBuilder does not seem to support DWARFv4 calculated
        // subranges yet.
        Nodecl::NodeclBase size = t.array_get_size();

        std::vector<llvm::Metadata*> subscripts;
        TL::Type current_type = t;
        while (current_type.is_fortran_array())
        {
            Nodecl::NodeclBase lower, upper;
            current_type.array_get_bounds(lower, upper);

            subscripts.push_back(dbg_builder->getOrCreateSubrange(
                lower.is_constant() ? const_value_cast_to_8(lower.get_constant()) : 0, 
                upper.is_constant() ? const_value_cast_to_8(upper.get_constant()) : 0));

            current_type = current_type.array_element();
        }

        llvm::DINodeArray array = dbg_builder->getOrCreateArray(subscripts);
        return dbg_builder->createArrayType(
                size.is_constant() ? const_value_cast_to_8(size.get_constant()) : 0,
                current_type.array_base_element().get_alignment_of(),
                get_debug_info_type(current_type.array_base_element()),
                array);
    }
    else if (t.is_fortran_array() && t.array_requires_descriptor())
    {
        // FIXME - LLVM is lacking in this area
        return dbg_builder->createUnspecifiedType("void");
    }
    else
    {
        internal_error("Cannot synthesize Debug Info type for type '%s'",
                       print_declarator(t.get_internal_type()));
    }
}

llvm::Value *FortranLLVM::gep_for_field(
    llvm::Type *struct_type,
    llvm::Value *addr,
    const std::vector<std::string> &access_fields)
{
    ERROR_CONDITION(access_fields.empty(), "Invalid empty set of fields", 0);

    llvm::Type *t = struct_type;
    std::vector<llvm::Value*> index_list;
    index_list.reserve(access_fields.size() + 1);
    index_list.push_back(get_integer_value_32(0));
    for (const auto &f : access_fields)
    {
        int idx = fields[t][f];

        index_list.push_back(get_integer_value_32(idx));

        t = llvm::cast<llvm::StructType>(t)->elements()[idx];
    }

    return ir_builder->CreateGEP(addr, index_list);
}

void FortranLLVM::initialize_llvm_types()
{
    llvm_types.i1 = llvm::Type::getInt1Ty(llvm_context);
    llvm_types.i8 = llvm::Type::getInt8Ty(llvm_context);
    llvm_types.i16 = llvm::Type::getInt16Ty(llvm_context);
    llvm_types.i32 = llvm::Type::getInt32Ty(llvm_context);
    llvm_types.i64 = llvm::Type::getInt64Ty(llvm_context);

    llvm_types.f32 = llvm::Type::getFloatTy(llvm_context);
    llvm_types.f64 = llvm::Type::getDoubleTy(llvm_context);

    llvm_types.void_ = llvm::Type::getVoidTy(llvm_context);

    llvm_types.ptr_i8 = llvm::Type::getInt8PtrTy(llvm_context);
    llvm_types.ptr_i32 = llvm::Type::getInt32PtrTy(llvm_context);
    llvm_types.ptr_i64 = llvm::Type::getInt64PtrTy(llvm_context);
}

}
