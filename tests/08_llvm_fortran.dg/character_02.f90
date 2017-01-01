! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="character_02.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    CHARACTER(LEN=6) :: C1
    CHARACTER(LEN=4) :: C2
    CHARACTER(LEN=10) :: C3

    C1 = "HELLO"
    C2 = "BYE"
    C3 = C1 // C2
    PRINT *, C3, "|"
END PROGRAM MAIN
