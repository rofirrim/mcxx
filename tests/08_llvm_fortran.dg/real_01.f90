! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="real_01.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    REAL :: A, B, C
    A = 10.0
    B = 4.0

    C = A + B
    PRINT *, C

    C = A - B
    PRINT *, C

    C = B - A
    PRINT *, C

    C = A * B
    PRINT *, C

    C = A / B
    PRINT *, C

    C = B / A
    PRINT *, C

    C = A ** B
    PRINT *, C

    C = B ** A
    PRINT *, C
END PROGRAM MAIN
