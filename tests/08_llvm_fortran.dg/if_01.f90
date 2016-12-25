! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="if.out"
! </testinfo>
PROGRAM MAIN
      PRINT *, "BEFORE"
      IF (.TRUE.) THEN
        PRINT *, "OK"
      ELSE
        PRINT *, "ERROR"
      END IF
      PRINT *, "AFTER"
END PROGRAM MAIN
