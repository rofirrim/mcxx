! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_04.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    INTEGER :: A(10, 10)
    INTEGER :: I, J

    DO J = 1, 10
        DO I = 1, 10
            A(I, J) = I - J
        END DO
    END DO
    PRINT *, A
END PROGRAM MAIN
