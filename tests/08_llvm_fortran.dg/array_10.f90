! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_10.f90.out"
! </testinfo>
MODULE MOO
CONTAINS
    SUBROUTINE S(X)
        IMPLICIT NONE
        INTEGER :: X(:)

        X = X + 1
        X = 1 + X
    END SUBROUTINE
END MODULE

PROGRAM MAIN
    USE MOO
    IMPLICIT NONE

    INTEGER :: A(10)
    INTEGER :: B(10)
    INTEGER :: I

    DO I = 1, 10
      A(I) = I
      B(I) = I + 2
    END DO

    CALL S(A)

    DO I = 1, 10
     IF (A(I) /= B(I)) THEN
         STOP 1
     END IF
    END DO

    PRINT *, A
    PRINT *, B

END PROGRAM MAIN
