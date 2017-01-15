! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_03.f90.out"
! </testinfo>
PROGRAM MAIN
    INTEGER, ALLOCATABLE :: A(:)

    ALLOCATE(A(3))

    A(1) = 1
    A(2) = 2
    A(3) = 3

    PRINT *, A

    DEALLOCATE(A)

END PROGRAM MAIN
