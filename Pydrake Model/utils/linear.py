##### PYDRAKE IMPORTS
from pydrake.systems.primitives import FirstOrderTaylorApproximation

##### OTHER IMPORTS
import numpy as np
import scipy.linalg as la

##### LINEARIZE FUNCTION (takes in diagram and context to find linearized system)
def LinearSys(system, context):
    # finding the linearized system (xd = Ax + Bu)
    return FirstOrderTaylorApproximation(system=system, context=context)

##### LINEAR SYSTEM OUTPUT FUNCTION (prints linear system quantities)
def LinearSysOut(linear_system) -> None:
    # finding A and B matrices
    A = linear_system.A()
    B = linear_system.B()

    print(f'\nLinearized Matrices:')
    print(f'A = {A}')
    print(f'\nB = {B}')

    # finding system parameters
    state_num = np.shape(A)[0] # number of rows/columns
    q_num = np.floor(state_num) # if odd, using quaternion representation

    # determining controllability
    C = np.array([])
    for i in range(q_num):
        Ctmp = np.linalg.matrix_power(A, i)
        if i == 0: C = Ctmp@B
        else: C = np.append(C, Ctmp@B, axis=1)

    rankA = np.linalg.matrix_rank(A)
    rankB = np.linalg.matrix_rank(B)
    rankC = np.linalg.matrix_rank(C)

    print(f'\nShape of A = {np.shape(A)}, Shape of B = {np.shape(B)}')
    print(f'Rank of A = {rankA}, Rank of B = {rankB}, Rank of C = {rankC}')

    # if not fully controllable
    if rankC != state_num: # if not full rank
        uncont_basis = la.null_space(C.T)
        print(f'\nUncontrollable Basis =\n', uncont_basis.T)

    # finding SVD decomposition
    _, CS, _ = la.svd(C)
    print(f'\nSingular Values of C =\n', CS)