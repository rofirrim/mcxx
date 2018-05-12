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

#include "timing.h"
#include <stdlib.h>
#include <string.h>

void timing_start(timing_t* t)
{
    memset(t, 0, sizeof(*t));
    
    gettimeofday(&(t->start), NULL);
}

void timing_end(timing_t* t)
{
    gettimeofday(&(t->end), NULL);

    double start_value = t->start.tv_sec*1e6 + t->start.tv_usec;
    double end_value = t->end.tv_sec*1e6 + t->end.tv_usec;

    double diff_value = end_value - start_value;

    t->elapsed_time = diff_value / 1e6;
}

double timing_elapsed(const timing_t* t)
{
    return (t->elapsed_time);
}


