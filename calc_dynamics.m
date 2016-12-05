%% Define rotation matrices
Rx = @(q) [1 0 0; 0 cos(q) -sin(q); 0 sin(q) cos(q)];
Ry = @(q) [cos(q) 0 sin(q); 0 1 0; -sin(q) 0 cos(q)];
Rz = @(q) [cos(q) -sin(q) 0; sin(q) cos(q) 0; 0 0 1];
DH = @(angle, offset, twist, length) [Rz(angle) [0; 0; offset]; 0 0 0 1] * [Rx(twist) [length; 0; 0]; 0 0 0 1];

%% Define shin-foot kinematic chain
% coordinates
syms x y z yaw pitch roll dp ie real
% derivatives
syms xd yd zd yawd pitchd rolld dpd ied real
syms xdd ydd zdd yawdd pitchdd rolldd dpdd iedd real

Q = [x y z yaw pitch roll dp ie]';
Qd = [xd yd zd yawd pitchd rolld dpd ied]';
Qdd = [xdd ydd zdd yawdd pitchdd rolldd dpdd iedd]';
F = sym('F', size(Q)); % actuator inputs

% frame transformations
R0s = Rz(yaw) * Ry(pitch) * Rx(roll); % inertial to shin frame
Rsf = Ry(dp) * Rx(ie); % shin to foot frame

% centers of gravity
rshing = [-5e-3; 1e-3; 20e-3]; % ankle to shin CG, on shin frame
rfootg = [80e-3; 5e-3; -30e-3]; % ankle to foot CG, on foot frame
ps = [x; y; z] + R0s * rshing; % shin CG position
pf = [x; y; z] + R0s * Rsf * rfootg; % foot CG position

% angular velocities
ws = [0; 0; yawd] + Rz(yaw)*([0; pitchd; 0] + Ry(pitch)*[rolld; 0; 0]);
% ws = R0s * [1 0 -sin(pitch); 0 cos(roll) cos(pitch)*sin(roll); 0 -sin(roll) cos(pitch)*cos(roll)] * [rolld; pitchd; yawd];
wf = ws + R0s * ([0; dpd; 0] + Ry(dp)*[ied; 0; 0]);
% wf = ws + R0s * Rsf * [1 0 -sin(dp); 0 cos(ie) cos(dp)*sin(ie); 0 -sin(ie) cos(dp)*cos(ie)] * [ied; dpd; 0];

% linear velocities
% vs = [xd; yd; zd] + cross(R0s' * ws, rshing);
% vf = [xd; yd; zd] + cross((R0s*Rsf)' * wf, rfootg);
vs = difft(ps, Q, Qd);
vf = difft(pf, Q, Qd);

% potential energy: gravitational only
g = [0; 0; -9.81];
ms = 8;
mf = 3;
V = ms * g' * ps + mf * g' * pf;

% kinetic energy
Js = diag([1, 1, 0.1]);
Jf = diag([0.02, 0.1, 0.1]);
T = ms * (vs.' * vs) / 2 + mf * (vf.' * vf) / 2 + ws.' * Js * ws / 2 + wf.' * Jf * wf / 2;

% Lagrangean dynamics
L = T - V;
eqs = difft(gradient(L, Qd), [Q; Qd], [Qd; Qdd]) - gradient(L, Q) == F;

eqs2 = difft(gradient(L, Qd), [Q; Qd], [Qd; Qdd]) - gradient(L, Q) - F;

% sol = solve(eqs, Qdd)

%% test code with pendulum

syms th real
syms thd real
syms thdd real

Q = [th];
Qd = [thd];
Qdd = [thdd];
F = sym('f', 1);

R01 = Rz(th);

% kinetics
Lbar = 1;
p = R01 * [Lbar/2; 0; 0];
v = difft(p, Q, Qd);

w = [0; 0; thd];

% potential energy
m = 5;
g = [0; 9.81; 0];
V = m * g.' * p;

% kinetic energy
J = m*Lbar^2/12;
T = simplify( 0.5 * m * (v.' * v) + 0.5 * w.' * J * w );

% Lagrangean dynamics
L = simplify(T - V);
b = 1;
Fnc = -b*thd;
eqs = simplify(difft(gradient(L, Qd), [Q; Qd], [Qd; Qdd]) - gradient(L, Q) == F + Fnc);
matlabFunction(solve(eqs, thdd), 'Vars', {Q, Qd, F}, 'File', 'dyn_func')

eq2 = simplify(thdd*(J + m*(Lbar/2)^2) == -0.5*m*Lbar*9.81*cos(th) - b*thd + F)
solve(eq2, thdd)

%% test code with two link robot

syms q1 q2 real
syms q1d q2d real
syms q1dd q2dd real

Q = [q1; q2];
Qd = [q1d; q2d];
Qdd = [q1dd; q2dd];
F = sym('f', [2, 1]);

l1 = 1.5;
l2 = 1;
T01 = DH(q1+pi/2, 0, 0, l1);
T12 = DH(q2-pi/2, 0, 0, l2);

% kinetics
p1 = T01 * [-l1/2; 0; 0; 1]; p1 = p1(1:3);
p2 = T01 * T12 * [-l2/2; 0; 0; 1]; p2 = p2(1:3);

v1 = difft(p1, Q, Qd);
v2 = difft(p2, Q, Qd);

w1 = [0; 0; q1d];
w2 = [0; 0; q1d + q2d];

% potential energy
m1 = 5;
m2 = 5;
g = [0; 9.81; 0];
V = m1 * g.' * p1 + m2 * g.' * p2;

% kinetic energy
rad1 = 0.1;
rad2 = 0.1;
J1 = diag([6*rad1^2/2, 3*rad1^2+l1^2, 3*rad1^2+l1^2] * m1/12);
J2 = diag([6*rad2^2/2, 3*rad2^2+l2^2, 3*rad2^2+l2^2] * m2/12);
T = simplify( 0.5 * m1 * (v1.' * v1) + 0.5 * w1.' * J1 * w1 + 0.5 * m2 * (v2.' * v2) + 0.5 * w2.' * J2 * w2 );

% Lagrangean dynamics
L = simplify(T - V);

% Non conservative forces
b = 10;
Fnc = -b*[q1d; q2d];

% External forces
T02 = T01 * T12;
X = T02(1:2,4); % [x; y]
J = jacobian(X, Q); % dX = J * dQ
Fa = sym('fa', [2 1]);

eqs = simplify(difft(gradient(L, Qd), [Q; Qd], [Qd; Qdd]) - gradient(L, Q) - Fnc == F + J'*Fa);

u_func = simplify(difft(gradient(L, Qd), [Q; Qd], [Qd; Qdd]) - gradient(L, Q) - Fnc - J'*Fa)
matlabFunction(u_func, 'Vars', {Q, Qd, Qdd, Fa}, 'File', 'u_func')

%%
% solve for higher derivative
sol = solve(eqs, Qdd);
matlabFunction([sol.q1dd; sol.q2dd], 'Vars', {Q, Qd, F, Fa}, 'File', 'dyn_func')

% test and set initial condition for simulation
dyn_func([0; 0], [0; 0], [0; 0])
Q0 = sin([0 2]);
Qd0 = sin([0 2]+pi/2);



subs(J, Q, zeros(2,1))' * [0; 1]




