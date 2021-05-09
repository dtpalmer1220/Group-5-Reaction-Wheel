# <div align="center">Group 5 - Inertial Pendulum </div>

#### <div align="center"><br>Hannah Canevaro</br><br>Tristan Patricio</br><br>Robert Perez-Cossio</br><br>Maria Rodriguez</br><br>Brandon Root</br><br>Allison Vang</br></div>

<div style="page-break-after: always;"></div>

# <b>Introduction:</b>

In this project, the Inertial Wheel Pendulum, or IWP, was investigated and modeled. The IWP utilizes the angular momentum of a mass to swing a pendulum bar up and balance the bar vertically. This project is a great practice in control systems. Utilizing a single motor and two sensors, the IWP will self balance upon having a force act upon it and changing its angle from zero to some other value. This will begin a swing up control and then balance the beam vertically, utilizing a motor attached to a rotatable mass. This system was first described in Spong et. Al. [1].

<div style="page-break-after: always;"></div>

# <b>Modeling:</b>

System Model and Related Calulations :(

<div style="page-break-after: always;"></div>

# <b>Sensor Calibration:</b>

Calibration (if needed) depending on simulation tool and sensor

<div style="page-break-after: always;"></div>

# <b>Sensor Controller Design and Simulations:</b>

Details of controller design

<div style="page-break-after: always;"></div>

# <b>OPTIONAL: Controller Implementation</b>

How can this be implemented on a real system?

<div style="page-break-after: always;"></div>

# <b>Appendix A - MATLab Code</b>
Variable Initialization:
clc,clear all;
% Motor parameters
R=4.172;
km=0.00775;
Umax=13;
% IWP Model
g=9.81;
mgl=0.12597;
mbg=mgl
d11=0.0014636;
d12=0.0000076;
d21=d12;
d22=d21;
J= (d11*d22-d12*d21)/d12;
D=[d11 d12;d21 d22];
Di=inv(D);
di11=Di(1,1)
di12=Di(1,2)
di21=Di(2,1)
di22=Di(2,2)
% Linear approximate model of IWP
A=[0 1 0;di11*mbg 0 0;di21*mbg 0 0]
B=[0;di12*km/R;di22*km/R]
% Controllability determination
disp('Is system controllable?');
Pc=ctrb(A,B);
if rank(Pc) == size(Pc)
disp('Yes.');
else
disp('No.');
end

% Nonlinear controller
Kd=20;
V0=2*mbg
d=(Umax*km)/(R*Kd*V0)

%Linear state feedback controller at the operation point
n=1
c=-570
xd=[n*pi 0 c]

% Desired closed-loop eigenvalues
lambda1= -9.27 + 20.6i;
lambda2= -9.27 - 20.6i;
lambda3= -0.719;
Vp=[lambda1 lambda2 lambda3]
K = place(A,B,Vp)
% Verifying closed-loop eigenvalues
Vp_=eig(A-B*K)

NonLinear Control
function [nlc,V,Verror] = fcn(x,d,V0)
%#codegen
V0=2*mbg
mgl=0.12597;
mbg=mgl;
R=4.172;
km=0.00775;
Kd=20;
d11=0.0014636;
d12=0.0000076;
d21=d12;
d22=d21;
J=(d11*d22-d12*d21)/d12;
if x(2)>d
satx2=d;
elseif x(2)<-d
satx2=-d;
else satx2=x(2);
end
%-----------------energy---------------------
V=(J/2)*x(2)^2+mbg*(1-cos(x(1)));
%------------nonlinear control---------------
nlc=(R/km)*Kd*satx2*(V-V0);
%-----------energy error---------------------
Verror=V-V0;

Linear Control:
function lc = fcn(x,K)
%#codegen}
n=1;
c=-570;
xd=[n*pi; 0; c];
z=x-xd;
%--------linear control-----------
lc =-K'*z;

Control Communitcation: 
function controller = fcn(nlc, lc, x)
%#codegen
n=1;
c=-570;
xd=[n*pi; 0; c];
z=x-xd;
if z(1)^2+z(2)^2<=0.01
%-----linear control action------------
controller=lc;
%-----nonlinear control action---------
else controller=nlc;
end


Simulink diagram, MATLAB and other code here

<div style="page-break-after: always;"></div>

# <b>References</b>
<br>[1] M. W. Spong, P. Corke, and R. Lozano, Nonlinear control of the reaction wheel pendulum,Automatica, vol. 37, no. 11, pp. 1845–1851, 2001</br>
<br>[2] V. M. Hernandez-Guzman and R. Silva-Ortigoza, Automatic Control with Experiments, Springer, 2019</br>
Sources used 
Utilize  https://scholar.google.com/  to obtain citations
