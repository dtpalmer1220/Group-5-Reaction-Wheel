clc, clear all, close all;

% Motor parameters (from vendor or references)
R=4.172;
km = 0.00775;
Umax = 13;

% Inertial wheel model
g = 9.81;                           % Gravitational acceleration
mgl = 0.12597;
mbg = mgl;
d11 = 0.0014636;
d12 = 0.0000076;
d21 = d12;
d22 = d21;

J = (d11*d22-d12*d21)/d12;
D = [d11 d12; d21 d22];
Di = inv(D);
di11 = Di(1,1);
di12 = Di(1,2);
di21 = Di(2,1);
di22 = Di(2,2);

% Linear approximate model of IWP
A = [ 0 1 0; di11*mbg 0 0; di21*mbg 0 0];
B = [0; di12*km/R; di22*km/R];

C = eye(size(A));

% Controllability for making sure of the application of 
% full-state feedback
disp('Is the system controllable?');
Pc = ctrb(A,B);
if rank(Pc) == size(Pc)
    disp('Yes.');
else
    disp('No.');
end

% Desired closed-loop eigenvalues (from requirements)
p1 = -9.27 + 20.6i;
p2 = -9.27 - 20.6i;
p3 = -0.719;
Vp = [ p1, p2, p3];
K = place(A, B, Vp);

% Verify closed-loop eigenvalues of A_new or A_cl
Vp_ = eig(A-B*K);


% Closed Loop Model
t = 0:0.01:5;
u = .001*ones(size(t));
x0 = [0.01,0,0];
sys_cl = ss(A-B*K,B,C,0);
lsim(sys_cl,u,t,x0)
xlabel('Time (sec)')
ylabel('Pendulum Position (Theta)')