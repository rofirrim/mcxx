! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="integer_02.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    INTEGER :: A, B, C
    A = 5
    B = 3

    C = A ** B

    PRINT *, C

    A = 5
    B = 0

    C = A ** B

    PRINT *, C

    A = 1
    B = -1

    C = A ** B

    PRINT *, C

    A = 1
    B = -2

    C = A ** B

    PRINT *, C

    A = 2
    B = -1

    C = A ** B

    PRINT *, C

    A = 2
    B = -2

    C = A ** B

    PRINT *, C
END PROGRAM MAIN
