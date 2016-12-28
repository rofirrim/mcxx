! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="do_02.f90.out"
! </testinfo>
PROGRAM MAIN
    DO I = 5, 1, -1
        PRINT *, I
    END DO
END PROGRAM MAIN
