! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="select_case_01.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    INTEGER :: I

    DO I = 1, 4
       SELECT CASE (I)
            CASE (1)
               PRINT *, "FIRST"
            CASE (2)
               PRINT *, "SECOND"
            CASE (3)
               PRINT *, "THIRD"
           CASE DEFAULT
               PRINT *, "FOURTH"
       END SELECT
    END DO
END PROGRAM MAIN
