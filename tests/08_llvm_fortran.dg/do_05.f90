! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="do_05.f90.out"
! </testinfo>
SUBROUTINE SUB(S)
    INTEGER :: I, S
    DO I = 1, 5, S
        PRINT *, I
    END DO
END SUBROUTINE SUB

PROGRAM MAIN
    CALL SUB(1)
    CALL SUB(2)
END PROGRAM MAIN
