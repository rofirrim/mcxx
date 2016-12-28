! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="do_06.f90.out"
! </testinfo>
SUBROUTINE SUB(S)
    INTEGER :: I, S
    DO I = 5, 1, S
        PRINT *, I
    END DO
END SUBROUTINE SUB

PROGRAM MAIN
    CALL SUB(-1)
    CALL SUB(-2)
END PROGRAM MAIN
