! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="if.out"
! </testinfo>
PROGRAM MAIN
      PRINT *, "BEFORE"
      IF (.FALSE.) THEN
        PRINT *, "ERROR"
      ELSE
        PRINT *, "OK"
      END IF
      PRINT *, "AFTER"
END PROGRAM MAIN
