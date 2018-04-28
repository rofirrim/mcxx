/*--------------------------------------------------------------------
  (C) Copyright 2006-2018 Barcelona Supercomputing Center
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




#include "tl-omp-core.hpp"
#include "tl-omp-deps.hpp"


namespace TL {

    template <>
    struct ModuleWriterTrait<OpenMP::DependencyItem::ItemDirection>
        : EnumWriterTrait<OpenMP::DependencyItem::ItemDirection> { };

    template <>
    struct ModuleReaderTrait<OpenMP::DependencyItem::ItemDirection>
        : EnumReaderTrait<OpenMP::DependencyItem::ItemDirection> { };

    namespace OpenMP {

    DependencyItem::DependencyItem(DataReference dep_expr, ItemDirection kind)
        : DataReference(dep_expr), _kind(kind)
    { }

    DependencyItem::ItemDirection DependencyItem::get_kind() const
    {
        return _kind;
    }

    void DependencyItem::module_write(ModuleWriter& mw)
    {
        this->TL::DataReference::module_write(mw);
        mw.write(_kind);
    }

    void DependencyItem::module_read(ModuleReader& mr)
    {
        this->TL::DataReference::module_read(mr);
        mr.read(_kind);
    }

} }
