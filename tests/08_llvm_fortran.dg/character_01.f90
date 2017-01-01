! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="character_01.f90.out"
! </testinfo>
PROGRAM MAIN
  IMPLICIT NONE
  CHARACTER(LEN=10) :: A

  A = "HELLO"
  PRINT *, A, "|"
  A = "BYE"
  PRINT *, A, "|"
END PROGRAM MAIN
