! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="do_04.f90.out"
! </testinfo>
PROGRAM MAIN
    DO I = 5, 1, -2
        PRINT *, I
    END DO
END PROGRAM MAIN
