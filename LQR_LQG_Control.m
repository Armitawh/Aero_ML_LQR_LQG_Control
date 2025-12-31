% Define symbolic variables
syms theta psi theta_dot psi_dot Vp Vy m l0 Jp Jy Dt Dp Dy Kpp Kpy Kyy Kyp g real

% Define state variables
x = [theta; psi; theta_dot; psi_dot];

% Define input variables
u = [Vp; Vy];

% Define nonlinear equations of motion
theta_ddot = (-m*cos(theta)*sin(theta)*l0*(psi_dot^2) - Dp*theta_dot - g*m*cos(theta)*l0 + Dt*(Kpy*Vy + Kpp*Vp)) / (m*l0^2 + Jp);
psi_ddot = (2*m*cos(theta)*sin(theta)*theta_dot*l0*psi_dot - Dy*psi_dot + Dt*(Kyy*Vy + Kyp*Vp)) / (m*l0^2*cos(theta)^2 + Jy);

% Define system equations
f = [theta_dot;
     psi_dot;
     theta_ddot;
     psi_ddot];

% Compute Jacobian matrices
A = jacobian(f, x); % Partial derivatives w.r.t. state variables
B = jacobian(f, u); % Partial derivatives w.r.t. input variables

% Substitute equilibrium point [theta = 0, psi = 0, theta_dot = 0, psi_dot = 0, l0 = 0]
A_lin = subs(A, [theta, psi, theta_dot, psi_dot, l0], [0, 0, 0, 0, 0]);
B_lin = subs(B, [theta, psi, theta_dot, psi_dot, l0], [0, 0, 0, 0, 0]);

% Define numerical values for parameters (adjust as needed)
param_values = [m, Jp, Jy, Dt, Dp, Dy, Kpp, Kpy, Kyy, Kyp, g];
num_values = [1.5, 0.02, 0.02, 0.1, 0.01, 0.01, 0.5, 0.2, 0.5, 0.2, 9.81]; % Replace with actual values

% Substitute numerical values
A_lin = subs(A_lin, param_values, num_values);
B_lin = subs(B_lin, param_values, num_values);

% Convert symbolic to numeric
A_lin = double(A_lin);
B_lin = double(B_lin);

% Display results
disp('Linearized System Matrices:');
disp('A = '); disp(A_lin);
disp('B = '); disp(B_lin);

K = lqr(A_lin, B_lin, Q, R);
disp('K = '); disp(K);