! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="do_03.f90.out"
! </testinfo>
PROGRAM MAIN
    DO I = 1, 5, 2
        PRINT *, I
    END DO
END PROGRAM MAIN
