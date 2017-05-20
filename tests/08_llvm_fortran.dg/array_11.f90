! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_11.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    INTEGER :: A(3), B(3)
    LOGICAL :: L(3)

    A(1) = 1
    A(2) = 2
    A(3) = 3

    B(1) = 3
    B(2) = 2
    B(3) = 1

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
