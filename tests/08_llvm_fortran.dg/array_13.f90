! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_13.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    INTEGER :: A(10), B(10)
    INTEGER :: I

    DO I = 1, 10
      A(I) = I
      B(I) = -I
    END DO

    A = -A

    PRINT *, A
    PRINT *, B

    DO I = 1, 10
      IF (A(I) /= B(I)) THEN
         PRINT *, I, "IS WRONG"
         STOP 1
     END IF
    END DO
END PROGRAM MAIN
