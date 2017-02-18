! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_07.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    INTEGER, ALLOCATABLE :: A(:), B(:)
    INTEGER :: N, I

    N = 4
    ALLOCATE(A(N))
    ALLOCATE(B(N))

    DO I = 1, N
      A(I) = I
      B(I) = -I
    END DO

    B = A

    PRINT *, A
    PRINT *, B
END PROGRAM MAIN
