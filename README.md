# <div align="center">Group 5 - Inertia Wheel Pendulum </div>

#### <div align="center"><br>Hannah Canevaro</br><br>Tristan Patricio</br><br>Robert Perez-Cossio</br><br>Maria Rodriguez</br><br>Brandon Root</br><br>Allison Vang</br></div>

<div style="page-break-after: always;"></div>

# <b>Introduction:</b>

In this project, the Inertial Wheel Pendulum, or IWP, was investigated and modeled. The IWP utilizes the angular momentum of a mass to swing a pendulum bar up and balance the bar vertically. This project is a great practice in control systems. Utilizing a single motor and two sensors, the IWP will self balance upon having a force act upon it and changing its angle from zero to some other value. This will begin a swing up control and then balance the beam vertically, utilizing a motor attached to a rotatable mass. This system was first described in Spong et. Al. [1].

<div style="page-break-after: always;"></div>

# <b>Modeling:</b>
A graphical representation of our model can be seen in the figure below.
<p align="center"><br>
<img src=https://www.researchgate.net/profile/Vijay-Muralidharan/publication/267559849/figure/fig3/AS:667641387679744@1536189519043/Schematic-of-the-inertia-wheel-pendulum-system.png>
</br></p>

<div style="page-break-after: always;"></div>

# <b>Sensor Calibration:</b>

Normally two position sensors would be used, however for this project, they will not be utilized and thus no calibration is needed. 

<div style="page-break-after: always;"></div>

# <b>Sensor Controller Design and Simulations:</b>
<b>Our system requires two controllers to be present in order to function properly. The first being a swing up control and the second being a balance control once the swing up has achieved its purpose. </br>
<br>The swing up control will bring the pendulum from a resting position, swing it back and forth utilizing the angular velocity of the rotating inertial mass until the angle of the pendumlums bar is vertical (theta = pi). This will then allow the balance control to take over. </br>
<br>Once the pendulum is vertical, the control will take over and maintain a theta close to pi in order to maintain a vertical position. This will last, still utilzing the inertial mass's angular velocity, until the system is turned off.</br>
<div style="page-break-after: always;"></div>

# <b>Controller Implementation</b>
<br> The Inertial Wheel Pendulum can potetially be designed as a sort of metronome or timing clock. By turning off and on at repeated intervals, an oscillitory motion could be obtained. This, along with a sort of proximity sensor can be used to simulate a type of timing clock within a greater system. </br>

<div style="page-break-after: always;"></div>

# <b>Appendix A - MATLab Code</b>
<br>Variable Initialization:</br>
<br>clc,clear all;</br>
<br>% Motor parameters</br>
<br>R=4.172;</br>
<br>km=0.00775;</br>
<br>Umax=13;</br>
<br>% IWP Model</br>
<br>g=9.81;</br>
<br>mgl=0.12597;</br>
<br>mbg=mgl</br>
<br>d11=0.0014636;</br>
<br>d12=0.0000076;</br>
<br>d21=d12;</br>
<br>d22=d21;</br>
<br>J= (d11*d22-d12*d21)/d12;</br>
<br>D=[d11 d12;d21 d22];</br>
<br>Di=inv(D);</br>
<br>di11=Di(1,1)</br>
<br>di12=Di(1,2)</br>
<br>di21=Di(2,1)</br>
<br>di22=Di(2,2)</br>
<br>% Linear approximate model of IWP</br>
<br>A=[0 1 0;di11*mbg 0 0;di21*mbg 0 0]</br>
<br>B=[0;di12*km/R;di22*km/R]</br>
<br>% Controllability determination</br>
<br>disp('Is system controllable?');</b>r
<br>Pc=ctrb(A,B);</br>
<br>if rank(Pc) == size(Pc)</br>
<br>disp('Yes.');</b>r
<br>else</br>
<br>disp('No.');</br>
<br>end</br>

<br>% Desired closed-loop eigenvalues</br>
<br>lambda1= -9.27 + 20.6i;</br>
<br>lambda2= -9.27 - 20.6i;</br>
<br>lambda3= -0.719;</br>
<br>Vp=[lambda1 lambda2 lambda3]</br>
<br>K = place(A,B,Vp)</br>

<br>% Verifying closed-loop eigenvalues</br>
<br>Vp_=eig(A-B*K)</br>

<br>% Close Loop Model</br>
<br>t = 0:0.01:5;</br>
<br>u = .001*ones(size(t));</br>
<br>x0 = [0.01,0,0];</br>
<br>sys_cl = ss(A-B*K,B,C,0);</br>
<br>lsim(sys_cl,u,t,x0)</br>
<br>xlabel('Time (sec)')</br>
<br>ylabel('Pendulum Position (Theta)')</br>





<br>All Code obtained from [2]</br>

Simulink diagram, MATLAB and other code here

<div style="page-break-after: always;"></div>

# <b>References</b>
<br>[1] M. W. Spong, P. Corke, and R. Lozano, Nonlinear control of the reaction wheel pendulum,Automatica, vol. 37, no. 11, pp. 1845–1851, 2001</br>
<br>[2] V. M. Hernandez-Guzman and R. Silva-Ortigoza, Automatic Control with Experiments, Springer, 2019</br>
Sources used 
Utilize  https://scholar.google.com/  to obtain citations
