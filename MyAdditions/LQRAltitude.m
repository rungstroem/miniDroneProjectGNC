%% Calculation of the LQR gain matrix [K] for the Altitude
% First define gravity and mass
g = 9.82;
m = 67; %grams
% Simplified system model
% m*zDotDot = -m*g + F

% State space representation
% xDot = A*x+B*u
% y = C*x
% xDot = [zDot zDotDot]^T
% x = [z zDot]^T
% A = [0 1; 0 0];
% B = [-g+1/m -g+1/m]^T
% C = [1 0];
A = [0 1; 0 0];
%B = [-g+(1/m) -g+(1/m)]';
B = [(1/m) (1/m)]';
C = [1 0];
D = 0;

% Create system from matrices
sys = ss(A,B,C,D);

% Define Q and R tuning matrices
Q = [0.01 0; 0 0.01];
R = 10;  % For this size system R can only be of dimensions 1x1 -scalar
% Calculate the LQR gain K
[KLQR S CLP] = lqr(sys, Q, R, 0);

% u = -Kx
% delta_u(t) = -K*delta_x(t)
% u(t) = -K*delta_x(t) + u^*
% u^* = I think is when u = g -> that is F = g - ie. the thrust should
% equal the gravity for equibrilibrium 