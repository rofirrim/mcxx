! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_compile_fail="yes"
! test_stdout="integer_02.f90.out"
! </testinfo>
PROGRAM MAIN
      IMPLICIT NONE
      INTEGER :: A
      INTEGER :: B
      INTEGER :: C

      A = 2
      B = 5

      C = A ** B
      PRINT *, C

      C = B ** A
      PRINT *, C
END PROGRAM MAIN
