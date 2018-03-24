/*--------------------------------------------------------------------
  (C) Copyright 2006-2012 Barcelona Supercomputing Center
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



/*
<testinfo>
test_generator=config/mercurium-ompss-cuda
</testinfo>
*/

#include <assert.h>

#pragma omp target device(cuda)
__global__ void addOne_gpu(int *a)
{
    *a += 1;
}

int main (int argc, char *argv[])
{
    int a = 1;

#pragma omp target device (cuda) copy_deps
#pragma omp task inout (a)
    {
        dim3 var;
        var.x = 1;
        var.y = 1;
        var.z = 1;
        addOne_gpu <<<1, 1>>> (&a);
    }

#pragma omp taskwait
    assert(a == 2);
    return 0;
}
