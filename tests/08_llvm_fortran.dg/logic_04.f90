! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="logic_04.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    LOGICAL :: L

    L = .TRUE.
    PRINT *, L

    L = .FALSE.
    PRINT *, L

END PROGRAM MAIN
