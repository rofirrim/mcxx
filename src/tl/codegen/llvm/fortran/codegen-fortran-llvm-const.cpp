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

namespace Codegen
{

llvm::Value *FortranLLVM::eval_constant(const_value_t *cval)
{
    if (const_value_is_integer(cval))
    {
        // FIXME: This should be elsewhere
        llvm::Type *t = nullptr;
        switch (const_value_get_bytes(cval))
        {
            case 1:
                t = llvm_types.i8;
            case 2:
                t = llvm_types.i16;
            case 4:
                t = llvm_types.i32;
            case 8:
                t = llvm_types.i64;
            default:
                internal_error("Code unreachable", 0);
        }
        return llvm::Constant::getIntegerValue(
            t,
            llvm::APInt(const_value_get_bytes(cval) * 8,
                        const_value_cast_to_8(cval),
                        const_value_is_signed(cval)));
    }
    else
    {
        internal_error("Constant evaluation of '%s' not implemented yet",
                       const_value_to_str(cval));
    }
}

llvm::Constant *FortranLLVM::get_integer_value_N(int64_t v,
                                                 llvm::Type *t,
                                                 int bits)
{
    return llvm::Constant::getIntegerValue(
        t, llvm::APInt(bits, v, /* signed */ 1));
}

llvm::Constant *FortranLLVM::get_integer_value(int64_t v, TL::Type t)
{
    ERROR_CONDITION(!t.is_signed_integral() && !t.is_bool(),
                    "Must be a signed integral or boolean",
                    0);
    return get_integer_value_N(v, get_llvm_type(t), t.get_size() * 8);
}

llvm::Constant *FortranLLVM::get_integer_value_32(int64_t v)
{
    return get_integer_value_N(v, llvm_types.i32, 32);
}

llvm::Constant *FortranLLVM::get_integer_value_64(int64_t v)
{
    return get_integer_value_N(int64_t(v), llvm_types.i64, 64);
}
}
