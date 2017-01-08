! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="character_04.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    CHARACTER(LEN=10) :: C

    C = "FOO"
    PRINT *, C == "BYE"
    PRINT *, C /= "BYE"
    PRINT *, C > "BYE"
    PRINT *, C >= "BYE"
    PRINT *, C < "BYE"
    PRINT *, C <= "BYE"

    PRINT *, C == "FOO"
    PRINT *, C /= "FOO"
    PRINT *, C > "FOO"
    PRINT *, C >= "FOO"
    PRINT *, C < "FOO"
    PRINT *, C <= "FOO"

    PRINT *, C == "FOO   "
    PRINT *, C /= "FOO   "
    PRINT *, C > "FOO   "
    PRINT *, C >= "FOO   "
    PRINT *, C < "FOO   "
    PRINT *, C <= "FOO   "

    C = "FOO "
    PRINT *, C == "BYE"
    PRINT *, C /= "BYE"
    PRINT *, C > "BYE"
    PRINT *, C >= "BYE"
    PRINT *, C < "BYE"
    PRINT *, C <= "BYE"

    PRINT *, C == "FOO"
    PRINT *, C /= "FOO"
    PRINT *, C > "FOO"
    PRINT *, C >= "FOO"
    PRINT *, C < "FOO"
    PRINT *, C <= "FOO"

    PRINT *, C == "FOO   "
    PRINT *, C /= "FOO   "
    PRINT *, C > "FOO   "
    PRINT *, C >= "FOO   "
    PRINT *, C < "FOO   "
    PRINT *, C <= "FOO   "

END PROGRAM MAIN
