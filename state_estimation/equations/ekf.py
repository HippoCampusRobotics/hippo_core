import sympy
import helper_functions as hf

symbol_mapping = dict(
    dt=dict(c='dt', latex='\Delta t'),
    g=dict(c='g', latex='g'),
    delta_angle_x=dict(c='delta_angle_x', latex='\Delta \Omega_x'),
    delta_angle_y=dict(c='delta_angle_y', latex='\Delta \Omega_y'),
    delta_angle_z=dict(c='delta_angle_z', latex='\Delta \Omega_z'),
    delta_angle=dict(c='delta_angle', latex='\Delta \Omega'),
    delta_velocity_x=dict(c='delta_velocity_x', latex='\Delta v_x'),
    delta_velocity_y=dict(c='delta_velocity_y', latex='\Delta v_y'),
    delta_velocity_z=dict(c='delta_velocity_z', latex='\Delta v_z'),
    delta_velocity=dict(c='delta_velocity', latex='\Delta v'),
    u=dict(c='u', latex='u'),
    delta_angle_x_var=dict(c='delta_angle_x_var',
                           latex='\sigma^2_{\Delta \Omega_x}'),
    delta_angle_y_var=dict(c='delta_angle_y_var',
                           latex='\sigma^2_{\Delta \Omega_y}'),
    delta_angle_z_var=dict(c='delta_angle_z_var',
                           latex='\sigma^2_{\Delta \Omega_z}'),
    delta_angle_var=dict(c='delta_angle_var',
                         latex='\sigma^2_{\Delta \Omega}'),
    delta_velocity_x_var=dict(c='delta_velocity_x_var',
                              latex='\sigma^2_{\Delta v_x}'),
    delta_velocity_y_var=dict(c='delta_velocity_y_var',
                              latex='\sigma^2_{\Delta v_y}'),
    delta_velocity_z_var=dict(c='delta_velocity_z_var',
                              latex='\sigma^2_{\Delta v_z}'),
    delta_velocity_var=dict(c='delta_velocity_var',
                            latex='\sigma^2_{\Delta v'),
    u_var=dict(c='u_var', latex='sigma^2_u'),
    qw=dict(c='qw', latex='q_w'),
    qx=dict(c='qx', latex='q_x'),
    qy=dict(c='qy', latex='q_y'),
    qz=dict(c='qz', latex='q_z'),
    q=dict(c='q', latex='q'),
    vx=dict(c='vx', latex='v_x'),
    vy=dict(c='vy', latex='v_y'),
    vz=dict(c='vz', latex='v_z'),
    v=dict(c='v', latex='v'),
    px=dict(c='px', latex='p^{ENU}_x'),
    py=dict(c='py', latex='p^{ENU}_y'),
    pz=dict(c='pz', latex='p^{ENU}_z'),
    p=dict(c='p', latex='p'),
    delta_angle_bias_x=dict(c='delta_angle_bias_x',
                            latex='\Delta \Omega_{bias, x}'),
    delta_angle_bias_y=dict(c='delta_angle_bias_y',
                            latex='\Delta \Omega_{bias, y}'),
    delta_angle_bias_z=dict(c='delta_angle_bias_z',
                            latex='\Delta \Omega_{bias, z}'),
    delta_angle_bias=dict(c='delta_angle_bias', latex='\Delta \Omega_{bias}'),
    delta_angle_true=dict(c='delta_angle_true', latex='\Delta \Omega_{true}'),
    delta_velocity_bias_x=dict(c='delta_velocity_bias_x',
                               latex='\Delta v_{bias, x}'),
    delta_velocity_bias_y=dict(c='delta_velocity_bias_y',
                               latex='\Delta v_{bias, y}'),
    delta_velocity_bias_z=dict(c='delta_velocity_bias_z',
                               latex='\Delta v_{bias, z}'),
    delta_velocity_bias=dict(c='delta_velocity_bias', latex='\Delta v_{bias}'),
    delta_velocity_true=dict(c='delta_velocity_true', latex='\Delta v_{true}'),
    state=dict(c='state', latex='x'),
    q_new=dict(c='q_new', latex='q_{k+1}'),
    v_new=dict(c='v_new', latex='v_{k+1}'),
    p_new=dict(c='p_new', latex='p_{k+1}'),
    delta_angle_bias_new=dict(c='delta_angle_bias_new',
                              latex='\Delta \Omega_{bias, k+1}'),
    delta_velocity_bias_new=dict(c='delta_velocity_bias_new',
                                 latex='\Delta v_{bias, k+1}'),
    state_new=dict(c='state_new', latex='x_{k+1}'),
    F=dict(c='F', latex='F'),
    G=dict(c='G', latex='G'),
    Q=dict(c='Q', latex='Q'),
    P=dict(c='P', latex='P'),
    P_new=dict(c='P_new', latex='P_{k+1}'),
    P_new_simplified=dict(c='P_new_simplified', latex='P_{k+1,simplified}'),
)

style = 'c'
sympy.init_printing()

dt = sympy.Symbol(symbol_mapping["dt"][style], real=True)
g = sympy.Symbol(symbol_mapping["g"][style], real=True)

delta_angle_x = sympy.Symbol(symbol_mapping["delta_angle_x"][style], real=True)
delta_angle_y = sympy.Symbol(symbol_mapping["delta_angle_y"][style], real=True)
delta_angle_z = sympy.Symbol(symbol_mapping["delta_angle_z"][style], real=True)
delta_angle = sympy.Matrix([delta_angle_x, delta_angle_y, delta_angle_z])
sympy.pprint(delta_angle)

delta_velocity_x = sympy.Symbol(symbol_mapping["delta_velocity_x"][style], real=True)
delta_velocity_y = sympy.Symbol(symbol_mapping["delta_velocity_y"][style], real=True)
delta_velocity_z = sympy.Symbol(symbol_mapping["delta_velocity_z"][style], real=True)
delta_velocity = sympy.Matrix([delta_velocity_x, delta_velocity_y, delta_velocity_z])
sympy.pprint(delta_velocity)

u = sympy.Matrix([delta_angle, delta_velocity])

delta_angle_x_var = sympy.Symbol(symbol_mapping["delta_angle_x_var"][style], real=True)
delta_angle_y_var = sympy.Symbol(symbol_mapping["delta_angle_y_var"][style], real=True)
delta_angle_z_var = sympy.Symbol(symbol_mapping["delta_angle_z_var"][style], real=True)
delta_angle_var = sympy.Matrix([delta_angle_x_var, delta_angle_y_var, delta_angle_z_var])

delta_velocity_x_var = sympy.Symbol(symbol_mapping["delta_velocity_x_var"][style], real=True)
delta_velocity_y_var = sympy.Symbol(symbol_mapping["delta_velocity_y_var"][style], real=True)
delta_velocity_z_var = sympy.Symbol(symbol_mapping["delta_velocity_z_var"][style], real=True)
delta_velocity_var = sympy.Matrix([delta_velocity_x_var, delta_velocity_y_var, delta_velocity_z_var])

u_var = sympy.Matrix.diag(delta_angle_x_var, delta_angle_y_var, delta_angle_z_var, delta_velocity_x_var, delta_velocity_y_var, delta_velocity_z_var)

qw = sympy.Symbol(symbol_mapping["qw"][style], real=True)
qx = sympy.Symbol(symbol_mapping["qx"][style], real=True)
qy = sympy.Symbol(symbol_mapping["qy"][style], real=True)
qz = sympy.Symbol(symbol_mapping["qz"][style], real=True)
q = sympy.Matrix([qw, qx, qy, qz])

R_to_earth = hf.quat2Rot(q)
R_to_body = R_to_earth.T

vx = sympy.Symbol(symbol_mapping["vx"][style], real=True)
vy = sympy.Symbol(symbol_mapping["vy"][style], real=True)
vz = sympy.Symbol(symbol_mapping["vz"][style], real=True)
v = sympy.Matrix([vx, vy, vz])

px = sympy.Symbol(symbol_mapping["px"][style], real=True)
py = sympy.Symbol(symbol_mapping["py"][style], real=True)
pz = sympy.Symbol(symbol_mapping["pz"][style], real=True)
p = sympy.Matrix([px, py, pz])

delta_angle_bias_x = sympy.Symbol(symbol_mapping["delta_angle_bias_x"][style], real=True)
delta_angle_bias_y = sympy.Symbol(symbol_mapping["delta_angle_bias_y"][style], real=True)
delta_angle_bias_z = sympy.Symbol(symbol_mapping["delta_angle_bias_z"][style], real=True)
delta_angle_bias = sympy.Matrix([delta_angle_bias_x, delta_angle_bias_y, delta_angle_bias_z])
delta_angle_true = delta_angle - delta_angle_bias

delta_velocity_bias_x = sympy.Symbol(symbol_mapping["delta_velocity_bias_x"][style], real=True)
delta_velocity_bias_y = sympy.Symbol(symbol_mapping["delta_velocity_bias_y"][style], real=True)
delta_velocity_bias_z = sympy.Symbol(symbol_mapping["delta_velocity_bias_z"][style], real=True)
delta_velocity_bias = sympy.Matrix([delta_velocity_bias_x, delta_velocity_bias_y, delta_velocity_bias_z])
delta_velocity_true = delta_velocity - delta_velocity_bias

state = sympy.Matrix([q, v, p, delta_angle_bias, delta_velocity_bias])

q_new = hf.quat_mult(q, sympy.Matrix([1, 0.5 * delta_angle_true[0], 0.5 * delta_angle_true[1], 0.5 * delta_angle_true[2]]))
v_new = v + R_to_earth * delta_velocity_true - sympy.Matrix([0, 0, g]) * dt
p_new = p + v * dt
delta_angle_bias_new = delta_angle_bias
delta_velocity_bias_new = delta_velocity_bias
state_new = sympy.Matrix([q_new, v_new, p_new, delta_angle_bias_new, delta_velocity_bias_new])
F = state_new.jacobian(state)
G = state_new.jacobian(u)
Q = G * u_var * G.T
P = hf.create_symmetric_cov_matrix([sympy.shape(state)[0], sympy.shape(state)[0]])

P_new = F * P * F.T + Q
# Remove lower triangle of P matrix
for row in range(sympy.shape(P_new)[0]):
    for col in range(sympy.shape(P_new)[1]):
        if row > col:
            P_new[row, col] = 0

P_new_simple = sympy.cse(P_new, sympy.utilities.iterables.numbered_symbols(prefix='t'), optimizations='basic')

with open("./generated/covariance_update_raw.c", "w") as f:
    hf.write_matrix(f, sympy.Matrix(P_new), "P_new", True)
with open("./generated/covariance_update.c", "w") as f:
    hf.write_subexpressions(f, P_new_simple[0])
    hf.write_matrix(f, sympy.Matrix(P_new_simple[1]), "P_new", True)
