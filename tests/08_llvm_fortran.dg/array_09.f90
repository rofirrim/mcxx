! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_09.f90.out"
! </testinfo>
MODULE MOO
    CONTAINS
SUBROUTINE SUB(A, B, C)
    IMPLICIT NONE
    REAL :: A(:), B(:), C(:)

    C = A + B
END SUBROUTINE SUB

END MODULE MOO

PROGRAM MAIN
    USE MOO
    IMPLICIT NONE
    INTEGER, PARAMETER :: N = 4
    REAL :: A(N), B(N), C(N)
    INTEGER :: I

    DO I = 1, N
      A(I) = I
      B(I) = I + 1
      C(I) = -I
    END DO

    CALL SUB(A, B, C)

    PRINT *, A
    PRINT *, B
    PRINT *, C
END PROGRAM MAIN
