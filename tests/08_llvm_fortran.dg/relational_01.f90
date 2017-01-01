! <testinfo>
! test_generator="config/mercurium-fortran-llvm run"
! test_stdout="relational_01.f90.out"
! </testinfo>
PROGRAM MAIN
    IMPLICIT NONE
    LOGICAL :: L
    INTEGER :: X, Y, Z

    X = 1
    Y = 2
    Z = 1

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
