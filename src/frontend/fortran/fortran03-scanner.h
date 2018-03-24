/*--------------------------------------------------------------------
  (C) Copyright 2006-2015 Barcelona Supercomputing Center
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


#ifndef FORTRAN03_NEW_SCANNER_H
#define FORTRAN03_NEW_SCANNER_H

#include "gperf-compat-types.h"

#include<stdio.h>

#include "cxx-macros.h"

MCXX_BEGIN_DECLS

// Used by fortran03-keywords.c
struct fortran_keyword_tag
{
    const char* name;
    int token_id;
};

extern struct fortran_keyword_tag * fortran_keywords_lookup (register const char *str, register gperf_length_t len);

MCXX_END_DECLS

#endif // FORTRAN03_NEW_SCANNER_H
