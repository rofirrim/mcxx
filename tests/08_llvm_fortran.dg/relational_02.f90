! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="relational_02.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    LOGICAL :: L
    REAL :: X, Y, Z

    X = 1.23
    Y = 2.45
    Z = 1.23

    ! > 
    L = X > Y
    PRINT *, L
    L = Y > X
    PRINT *, L
    L = X > X
    PRINT *, L
    L = X > Z
    PRINT *, L

    ! <
    L = X < Y
    PRINT *, L
    L = Y < X
    PRINT *, L
    L = X < X
    PRINT *, L
    L = X < Z
    PRINT *, L

    ! <=
    L = X <= Y
    PRINT *, L
    L = Y <= X
    PRINT *, L
    L = X <= X
    PRINT *, L
    L = X <= Z
    PRINT *, L

    ! >=
    L = X >= Y
    PRINT *, L
    L = Y >= X
    PRINT *, L
    L = X >= X
    PRINT *, L
    L = X >= Z
    PRINT *, L

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
