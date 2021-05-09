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
<b>Variable Initialization:</b>
<b>clc,clear all;</b>
<b>% Motor parameters</b>
<b>R=4.172;</b>
<b>km=0.00775;</b>
<b>Umax=13;</b>
<b>% IWP Model</b>
<b>g=9.81;</b>
<b>mgl=0.12597;</b>
<b>mbg=mgl</b>
<b>d11=0.0014636;</b>
<b>d12=0.0000076;</b>
<b>d21=d12;</b>
<b>d22=d21;</b>
<b>J= (d11*d22-d12*d21)/d12;</b>
<b>D=[d11 d12;d21 d22];</b>
<b>Di=inv(D);</b>
<b>di11=Di(1,1)</b>
<b>di12=Di(1,2)</b>
<b>di21=Di(2,1)</b>
<b>di22=Di(2,2)</b>
<b>% Linear approximate model of IWP</b>
<b>A=[0 1 0;di11*mbg 0 0;di21*mbg 0 0]</b>
<b>B=[0;di12*km/R;di22*km/R]</b>
<b>% Controllability determination</b>
<b>disp('Is system controllable?');</b>
<b>Pc=ctrb(A,B);</b>
<b>if rank(Pc) == size(Pc)</b>
<b>disp('Yes.');</b>
<b>else</b>
<b>disp('No.');</b>
<b>end</b>

<b>% Nonlinear controller</b>
<b>Kd=20;</b>
<b>V0=2*mbg</b>
<b>d=(Umax*km)/(R*Kd*V0)</b>

<b>%Linear state feedback controller at the operation point</b>
<b>n=1</b>
<b>c=-570</b>
<b>xd=[n*pi 0 c]</b>

<b>% Desired closed-loop eigenvalues</b>
<b>lambda1= -9.27 + 20.6i;</b>
<b>lambda2= -9.27 - 20.6i;</b>
<b>lambda3= -0.719;</b>
<b>Vp=[lambda1 lambda2 lambda3]</b>
<b>K = place(A,B,Vp)</b>
<b>% Verifying closed-loop eigenvalues</b>
<b>Vp_=eig(A-B*K)</b>

<b>NonLinear Control</b>
<b>function [nlc,V,Verror] = fcn(x,d,V0)</b>
<b>%#codegen</b>
<b>V0=2*mbg</b>
<b>mgl=0.12597;</b>
<b>mbg=mgl;</b>
<b>R=4.172;</b>
<b>km=0.00775;</b>
<b>Kd=20;</b>
<b>d11=0.0014636;</b>
<b>d12=0.0000076;</b>
<b>d21=d12;</b>
<b>d22=d21;</b>
<b>J=(d11*d22-d12*d21)/d12;</b>
<b>if x(2)>d</b>
<b>satx2=d;</b>
<b>elseif x(2)<-d</b>
<b>satx2=-d;</b>
<b>else satx2=x(2);</b>
<b>end</b>
<b>%-----------------energy---------------------</b>
<b>V=(J/2)*x(2)^2+mbg*(1-cos(x(1)));</b>
<b>%------------nonlinear control---------------</b>
<b>nlc=(R/km)*Kd*satx2*(V-V0);</b>
<b>%-----------energy error---------------------</b>
<b>Verror=V-V0;</b>

<b>Linear Control:</b>
<b>function lc = fcn(x,K)</b>
<b>%#codegen}</b>
<b>n=1;</b>
<b>c=-570;</b>
<b>xd=[n*pi; 0; c];</b>
<b>z=x-xd;</b>
<b>%--------linear control-----------</b>
<b>lc =-K'*z;</b>

<b>Control Communitcation: </b>
<b>function controller = fcn(nlc, lc, x)</b>
<b>%#codegen</b>
<b>n=1;</b>
<b>c=-570;</b>
<b>xd=[n*pi; 0; c];</b>
<b>z=x-xd;</b>
<b>if z(1)^2+z(2)^2<=0.01</b>
<b>%-----linear control action------------</b>
<b>controller=lc;</b>
<b>%-----nonlinear control action---------</b>
<b>else controller=nlc;</b>
<b>end</b>


Simulink diagram, MATLAB and other code here

<div style="page-break-after: always;"></div>

# <b>References</b>
<br>[1] M. W. Spong, P. Corke, and R. Lozano, Nonlinear control of the reaction wheel pendulum,Automatica, vol. 37, no. 11, pp. 1845–1851, 2001</br>
<br>[2] V. M. Hernandez-Guzman and R. Silva-Ortigoza, Automatic Control with Experiments, Springer, 2019</br>
Sources used 
Utilize  https://scholar.google.com/  to obtain citations
