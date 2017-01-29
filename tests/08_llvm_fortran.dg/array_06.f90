! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_06.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    INTEGER :: A(3)
    INTEGER :: B(3)
    INTEGER :: C(3)
    INTEGER :: T(3)
    INTEGER :: I

    DO I = 1, 3
       A(I) = 2*I
       B(I) = I
       T(I) = 3*I
    END DO

    C = A + B

    PRINT *, A
    PRINT *, B
    PRINT *, C
    PRINT *, T
END PROGRAM MAIN
