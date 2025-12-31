% -------------------------------------------------------------------------
% USE THIS SCRIPT TO COMPLETE THE LQR/LQG LAB SESSION 
% Some commands are commented out by default which you can uncomment as
% necessary.
% -------------------------------------------------------------------------

%% CLEAR THE WORKSPACE & LOAD AERO2 PARAMETERS
clear

% LOAD AERO2 PARAMETERS
aero2_parameters;


%% DEFINE AERO 2 STATE SPACE MATRICES
A = [0 0 1 0; 0 0 0 1; -Ksp/Jp 0 -Dp/Jp 0; 0 0 0 -Dy/Jy];
B = [0 0; 0 0; (Dt*Kpp)/Jp (Dt*Kpy)/Jp; (Dt*Kyp)/Jy (Dt*Kyy)/Jy];
C = [1 0 0 0; 0 1 0 0];
D = [0 0; 0 0];


%% DESIGN LQR GAIN K (TASK 3)
Q = diag([150 70 120 40]); % ---Tune this
R = 0.04*diag([1 1]); % ---Tune this

K = lqr(A,B,Q,R);

% Checkout the closed loop poles
lqr_closed_loop_poles = eig(A-B*K);

% Uncomment to save controller to file
% save lqr_controller1.mat K

%% LQR WITH INTEGRAL ACTION (TASK 5)

% DEFINE AUGMENTED STATE SPACE MATRICES WITH 2 ADDITIONAL STATES

% Augmented A matrix
Aa = [A zeros(4,2);
      eye(2,2) zeros(2,4)];
% Augmented B matrix
Ba = [B; 
      zeros(2,2)];
% B matrix related to reference variables
Bb = [zeros(4,2); 
      -eye(2,2)];
% Augmented C matrix
Ca = [C zeros(2,2)];

% DESIGN OF AUGMENETED CONTROLLER Ka
Qa = diag([4955250 90000 111220 1004 50 1000000]); % --- Tune this
Ra = diag([1 1]); % --- Tune this

Ka = lqr(Aa, Ba, Qa, Ra);

% Uncomment to save the controller to file
save lqr_integral_controller1.mat Ka


%% LQG CONTROLLER DESIGN

% Design the observer gain
Q_Bar = diag([1 2 3 4]); % ---Tune this
R_Bar = diag([1 1]); % ---Tune this

L = transpose(lqr(A',C',Q_Bar,R_Bar));

% checkout the observer poles
obs_closed_loop_poles = eig(A-L*C);

% Uncomment to save final gains for both controller/observer
% save lqg_controller1.mat Ka L

%% CHECK SINGULAR VALUES
% figure
% ----Do singular value plot here-----

% SAVE CONTROLLER GAINS K and L
% save controller.mat K L

