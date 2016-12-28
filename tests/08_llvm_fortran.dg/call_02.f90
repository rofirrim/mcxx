! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="call_02.f90.out"
! </testinfo>
SUBROUTINE SUB(N)
  IMPLICIT NONE
  INTEGER :: N

  PRINT *, N
  N = N + 1
END SUBROUTINE SUB

PROGRAM MAIN
  IMPLICIT NONE
  INTEGER :: X

  X = 10
  CALL SUB(X)
  PRINT *, X
END PROGRAM MAIN
