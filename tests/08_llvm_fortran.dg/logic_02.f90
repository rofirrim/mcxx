! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! </testinfo>
PROGRAM MAIN
      LOGICAL :: L
      L = .FALSE.
      IF (L) THEN
        STOP 1
      ELSE
        CONTINUE
      END IF
END PROGRAM MAIN
