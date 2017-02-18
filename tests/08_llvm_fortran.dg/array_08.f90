! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_08.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    INTEGER, ALLOCATABLE :: A(:), B(:), C(:)
    INTEGER :: N, I

    N = 4
    ALLOCATE(A(N))
    ALLOCATE(B(N))
    ALLOCATE(C(N))

    DO I = 1, N
      A(I) = 1 + I
      B(I) = 2 + I
      C(I) = -I
    END DO

    C = A + B
    PRINT *, A
    PRINT *, B
    PRINT *, C
END PROGRAM MAIN
