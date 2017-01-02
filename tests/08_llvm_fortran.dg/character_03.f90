! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="character_03.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    CHARACTER(LEN=10) :: A, B

    A = "ABC"
    B = "ABC"
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B

    A = "ABC"
    B = "ABD"
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B

    A = "ABD"
    B = "ABC"
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B

    A = "ABC"
    B = "DEF"
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B

    A = "DEF"
    B = "ABC"
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B

    A = "ABC"
    B = "ABC  "
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B

    A = "ABC"
    B = "ABD  "
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B

    A = "ABC   "
    B = "ABC  D"
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B

    A = "ABC   "
    B = "ABC   "
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B

    A = "ABC   "
    B = "ABC"
    PRINT *, A < B
    PRINT *, A > B
    PRINT *, A <= B
    PRINT *, A >= B
    PRINT *, A == B
    PRINT *, A /= B
END PROGRAM MAIN

