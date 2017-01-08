! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="relational_04.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    LOGICAL :: L1, L2

    L1 = .TRUE.
    L2 = .TRUE.
    PRINT *, L1 .EQV. L2
    PRINT *, L1 .NEQV. L2

    L1 = .TRUE.
    L2 = .FALSE.
    PRINT *, L1 .EQV. L2
    PRINT *, L1 .NEQV. L2

    L1 = .FALSE.
    L2 = .TRUE.
    PRINT *, L1 .EQV. L2
    PRINT *, L1 .NEQV. L2

    L1 = .FALSE.
    L2 = .FALSE.
    PRINT *, L1 .EQV. L2
    PRINT *, L1 .NEQV. L2
END PROGRAM MAIN
