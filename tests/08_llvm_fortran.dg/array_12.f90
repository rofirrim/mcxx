! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_12.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    REAL :: A(3), B(3)
    LOGICAL :: L(3)

    A(1) = 1.2
    A(2) = 2.3
    A(3) = 3.4

    B(1) = 3.4
    B(2) = 2.3
    B(3) = 1.2

    L = A == B
    PRINT *, L

    L = A /= B
    PRINT *, L

    L = A > B
    PRINT *, L

    L = A < B
    PRINT *, L

    L = A >= B
    PRINT *, L

    L = A <= B
    PRINT *, L
END PROGRAM MAIN
