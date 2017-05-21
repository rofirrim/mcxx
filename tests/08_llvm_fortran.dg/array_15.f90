! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_15.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    LOGICAL :: A(2), B(2), C(2)

    A(1) = .TRUE.
    A(2) = .FALSE.

    B(1) = .FALSE.
    B(2) = .TRUE.

    PRINT *, A
    PRINT *, B

    C = .NOT. A

    PRINT *, C
END PROGRAM MAIN
