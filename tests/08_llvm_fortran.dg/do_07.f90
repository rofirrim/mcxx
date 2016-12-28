! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="do_07.f90.out"
! </testinfo>
SUBROUTINE SUB(L, U, S)
    INTEGER :: I, L, U, S
    DO I = L, U, S
        PRINT *, I
    END DO
END SUBROUTINE SUB

PROGRAM MAIN
    CALL SUB(1, 5, 1)
    CALL SUB(1, 5, 2)
    CALL SUB(5, 1, -1)
    CALL SUB(5, 1, -2)
END PROGRAM MAIN
