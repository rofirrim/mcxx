! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="array_16.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    LOGICAL :: A(4), B(4), C(4)


    A(1) = .TRUE. ; B(1) = .TRUE.
    A(2) = .TRUE. ; B(2) = .FALSE.
    A(3) = .FALSE. ; B(3) = .TRUE.
    A(4) = .FALSE. ; B(4) = .FALSE.

    C = A .EQV. B
    PRINT *, C

    C = A .NEQV. B
    PRINT *, C
END PROGRAM MAIN
