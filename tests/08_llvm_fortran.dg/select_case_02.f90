! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="select_case_02.f90.out"
! </testinfo>
MODULE MOO
    CONTAINS
        SUBROUTINE SUB(X)
            CHARACTER(LEN=10) :: X
            SELECT CASE(X)
                CASE ("HI")
                    PRINT *, "HELLO!"
                CASE ("BYE")
                    PRINT *, "GOOD BYE!"
                CASE DEFAULT
                    PRINT *, "SAY THAT AGAIN"
            END SELECT
        END SUBROUTINE SUB
END MODULE MOO

PROGRAM MAIN
    USE MOO
    IMPLICIT NONE
    CHARACTER(LEN=10) :: C

    C = "HI"
    CALL SUB(C)

    C = "BYE"
    CALL SUB(C)

    C = "FOO"
    CALL SUB(C)
END PROGRAM MAIN
