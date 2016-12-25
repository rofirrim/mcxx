! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="integer_01.f90.out"
! </testinfo>
PROGRAM MAIN
      IMPLICIT NONE
      INTEGER :: A
      INTEGER :: B
      INTEGER :: C

      A = 2
      PRINT *, A
      B = 5
      PRINT *, B

      C = A + B
      PRINT *, C

      C = B - A
      PRINT *, C

      C = A * B
      PRINT *, C

      C = B * A
      PRINT *, C

      C = B / A
      PRINT *, C

      C = A / B
      PRINT *, C
    
      C = -B
      PRINT *, C

      C = -B / A
      PRINT *, C

      C = B / (-A)
      PRINT *, C
END PROGRAM MAIN
