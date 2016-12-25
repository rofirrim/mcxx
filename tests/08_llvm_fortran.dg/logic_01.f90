! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! </testinfo>
PROGRAM MAIN
      LOGICAL :: L
      L = .TRUE.
      IF (L) THEN
        CONTINUE
      ELSE
        STOP 1
      END IF
END PROGRAM MAIN
