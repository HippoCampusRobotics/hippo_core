import sympy
from sympy.codegen.ast import real, float64


def quat_mult(p, q):
    r = sympy.Matrix([
        p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
        p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
        p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
        p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]
    ])

    return r


def quat2Rot(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    Rot = sympy.Matrix([[
        1 - 2 * q2**2 - 2 * q3**2, 2 * (q1 * q2 - q0 * q3),
        2 * (q1 * q3 + q0 * q2)
    ],
                        [
                            2 * (q1 * q2 + q0 * q3), 1 - 2 * q1**2 - 2 * q3**2,
                            2 * (q2 * q3 - q0 * q1)
                        ],
                        [
                            2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1),
                            1 - 2 * q1**2 - 2 * q2**2
                        ]])

    return Rot


def create_cov_matrix(i, j):
    if j >= i:
        return sympy.Symbol("P_(" + str(i) + "," + str(j) + ")", real=True)
        # legacy array format
        # return Symbol("P[" + str(i) + "][" + str(j) + "]", real=True)
    else:
        return 0


def create_symmetric_cov_matrix(size):
    # define a symbolic covariance matrix
    P = sympy.Matrix(size[0], size[1], create_cov_matrix)

    for index in range(size[0]):
        for j in range(size[0]):
            if index > j:
                P[index, j] = P[j, index]

    return P


def write_subexpressions(filehandle, expressions):
    output = ""
    for x in expressions:
        tmp = sympy.ccode(x[1], assign_to=x[0], type_aliases={real: float64})
        output += f"const double {tmp}\n"
    filehandle.write(output)


def write_matrix(filehandle,
                 matrix,
                 variable_name,
                 symmetric=False,
                 pre_bracket='(',
                 post_bracket=')'):

    def get_2d_name(i, j):
        return f"{variable_name}{pre_bracket}{i}, {j}{post_bracket}"

    output = ""
    if matrix.shape[0] * matrix.shape[1] == 1:
        tmp = sympy.ccode(matrix[0],
                          assign_to=variable_name,
                          type_aliases={real: float64})
        output = f"{tmp}\n"
    elif matrix.shape[0] == 1 or matrix.shape[1] == 1:
        for i in range(len(matrix)):
            tmp = sympy.ccode(
                matrix[i],
                assign_to=f"{variable_name}{pre_bracket}{i}{post_bracket}",
                type_aliases={real: float64})
            output += f"{tmp}\n"
    else:
        for col in range(matrix.shape[1]):
            for row in range(matrix.shape[0]):
                if col >= row or not symmetric:
                    tmp = sympy.ccode(matrix[row, col],
                                      assign_to=get_2d_name(row, col),
                                      type_aliases={real: float64})
                    output += f"{tmp}\n"
    filehandle.write(output)
