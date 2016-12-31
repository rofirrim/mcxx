! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="complex_02.f90.out"
! </testinfo>
PROGRAM MAIN
  IMPLICIT NONE
  COMPLEX :: C, D, E

  C = (2, 3)
  D = (4, 5)

  E = C ** D
  PRINT *, E

  E = D ** C
  PRINT *, E
END PROGRAM MAIN
