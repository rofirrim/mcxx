! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="complex_01.f90.out"
! </testinfo>
PROGRAM MAIN
  IMPLICIT NONE
  COMPLEX :: C, D, E

  C = (4, 3)
  D = (8, 6)

  E = C + D
  PRINT *, E

  E = C - D
  PRINT *, E

  E = D - C
  PRINT *, E

  E = D * C
  PRINT *, E

  E = C * D
  PRINT *, E

  E = C / D
  PRINT *, E

  E = D / E
  PRINT *, E
END PROGRAM MAIN
