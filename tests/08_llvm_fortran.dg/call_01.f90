! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="call_01.f90.out"
! </testinfo>
MODULE MOO
  IMPLICIT NONE
  CONTAINS
    SUBROUTINE SUB(N)
      INTEGER :: N
    
      PRINT *, N
      N = N + 1
    END SUBROUTINE SUB
END MODULE MOO

PROGRAM MAIN
  USE MOO
  IMPLICIT NONE
  INTEGER :: X

  X = 10
  CALL SUB(X)
  PRINT *, X
END PROGRAM MAIN
