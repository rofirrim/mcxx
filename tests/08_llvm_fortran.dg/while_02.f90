! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="while_02.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    INTEGER :: I

    I = 10
    DO WHILE (I < 10)
        PRINT *, I
        I = I + 1
    END DO
END PROGRAM MAIN
