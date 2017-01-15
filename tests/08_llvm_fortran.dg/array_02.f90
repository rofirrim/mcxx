! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_02.f90.out"
! </testinfo>
PROGRAM MAIN
    REAL :: A(3)

    A(1) = 1.2
    A(2) = 2.3
    A(3) = 3.4

    PRINT *, A

END PROGRAM MAIN
