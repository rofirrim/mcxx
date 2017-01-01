! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="relational_03.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    LOGICAL :: L
    COMPLEX :: X, Y, Z

    X = (1.23, 3.45)
    Y = (5.67, 8.90)
    Z = (1.23, 3.45)

    ! ==
    L = X == Y
    PRINT *, L
    L = Y == X
    PRINT *, L
    L = X == X
    PRINT *, L
    L = X == Z
    PRINT *, L

    ! /=
    L = X /= Y
    PRINT *, L
    L = Y /= X
    PRINT *, L
    L = X /= X
    PRINT *, L
    L = X /= Z
    PRINT *, L
END PROGRAM MAIN
