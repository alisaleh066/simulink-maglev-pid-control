clear all
clc
syms x1 x2 x3 u s tt II AA

%% Equilibrium Points (4)
R = 50;
L = 0.2;
g = 9.8;
M = 0.491;
c = 0.3;
fv = 0.04;

eq1 = x2 == 0;
eq2 = x1 == 0.06;
eq3 = -g + (c/M)*(x3^2/(0.1-x1)) - (fv*x2/M) == 0;
eq4 = (1/L)*(-R*x3 + u) == 0;

stateEqs = [eq1, eq2, eq3, eq4];
sol = solve(stateEqs);

xe1 = sol.x1;
xe2 = sol.x2;
xe3 = sol.x3;

%% Linearization (5)
f = x2;
g = -g + (c/M)*(x3^2/(0.1-x1)) - (fv*x2/M);
h = (1/L)*(-R*x3 + u);

A = jacobian([f, g, h], [x1, x2, x3]);

A_1 = subs(A, {x1, x2, x3}, {xe1(1), xe2(1), xe3(1)});
A_2 = subs(A, {x1, x2, x3}, {xe1(2), xe2(2), xe3(2)});

eigA1 = eig(A_1);
eigA2 = eig(A_2);

%% State‑Space Analysis (6)
% Controllability
B = [0; 0; 5];

Phi_c1 = [B, A_1*B, A_1^2*B];
Phi_c2 = [B, A_2*B, A_2^2*B];

det_Phi_c1 = det(Phi_c1);
det_Phi_c2 = det(Phi_c2);

% Observability
C = [1 0 0];

Phi_o1 = [C; C*A_1; C*A_1^2];
Phi_o2 = [C; C*A_2; C*A_2^2];

rank_Phi_o1 = rank(Phi_o1);
rank_Phi_o2 = rank(Phi_o2);

%% Transfer Function (7)
g1 = C * inv(s*eye(size(A_1)) - A_1) * B;
g2 = C * inv(s*eye(size(A_2)) - A_2) * B;

poleq = 491*s^3 + 122790*s^2 - 110295*s - 30073750;
poles = solve(poleq, s);

%% PID Controller (8)
s = tf('s');

G1 = -60073.7 / (491*s^3 + 122790*s^2 - 110295*s - 30073750);
G2 =  60073.7 / (491*s^3 + 122790*s^2 - 110295*s - 30073750);

t = 0:0.01:2;
figure; step(G1, t); title('Step Response Open Loop G1');
figure; step(G2, t); title('Step Response Open Loop G2');

% P
Kp = 1600;
C = pid(Kp);
T1 = feedback(C*G1,1);
T2 = feedback(C*G2,1);
figure; step(T1, t); title('Closed‑Loop with P on G1');
figure; step(T2, t); title('Closed‑Loop with P on G2');

% PD
Kp = 1600; Kd = 500;
C  = pid(Kp, 0, Kd);
T3 = feedback(C*G2, 1);
figure; stepinfo(T3);
figure; step(T3, t); title('Closed‑Loop with PD on G2');

% PID
Kp = 1600; Ki = 2600; Kd = 245;
C  = pid(Kp, Ki, Kd);
T4 = feedback(C*G2, 1);
figure; stepinfo(T4);
figure; step(T4, t); title('Closed‑Loop with PID on G2');
