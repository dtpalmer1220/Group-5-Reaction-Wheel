# <div align="center">Group 5 - Inertia Wheel Pendulum </div>

#### <div align="center"><br>Hannah Canevaro</br><br>Tristan Patricio</br><br>Robert Perez-Cossio</br><br>Maria Rodriguez</br><br>Brandon Root</br><br>Allison Vang</br></div>

<div style="page-break-after: always;"></div>

# <b>Introduction:</b>
A link to the presentation of this project can be found here: 
https://github.com/broot97/Group-5-Reaction-Wheel/blob/main/Presentation.mp4

In this project, the Inertial Wheel Pendulum, or IWP, was investigated and modeled. The IWP utilizes the angular momentum of a mass to swing a pendulum bar up and balance the bar vertically. This project is a great practice in control systems. Utilizing a single motor and two sensors, the IWP will self balance upon having a force act upon it and changing its angle from zero to some other value. This will begin a swing up control and then balance the beam vertically, utilizing a motor attached to a rotatable mass. This system was first described in Spong et. Al. [1]. This system has two degrees of freedom, one on the pendulum and one on the actuator iteself.

A capabilities database and a functional viewpoint were created at the beginning of the project. These helped understand the process that we need to model in order to proceed. This step was extremely valuable in taking steps in the right direction. 

<p align="center"><br>
<img src=https://user-images.githubusercontent.com/79555262/118375644-d5f0dd00-b577-11eb-84f2-70351b16cded.png
</br>
<br>Capabilities Database of the Inertial Wheel Pendulum</br>
</p>

<p align="center"><br>
<img src=https://user-images.githubusercontent.com/79555262/118375650-dc7f5480-b577-11eb-9885-2b3c2d288427.png
</br>
<br>Functional Viewpoint of the Inertial Wheel Pendulum</br>
</p>

Utilizing Spong et. Al. [1] and Hernandez-Guzman [2] we were able to begin to model the system and determine if the model is controllable.
<div style="page-break-after: always;"></div>

# <b>Modeling:</b>
A visual representation of our model can be seen in the figure below.
<p align="center"><br>
<img src=https://www.researchgate.net/profile/Vijay-Muralidharan/publication/267559849/figure/fig3/AS:667641387679744@1536189519043/Schematic-of-the-inertia-wheel-pendulum-system.png>
</br></p>

This image can be used to determine the matrices needed to control the system. Utilzing dynamic and control systems principals the following matrices were found that represent a linear model of our system. The following matrices, from Hernandez-Guzman [2], were the matrices used to control our system.

<p align="center"><br>
<img src=https://user-images.githubusercontent.com/79555262/118376163-2f0e4000-b57b-11eb-9678-badce0111ee2.png
</br></p>
<p align="center"><br>
<img src=https://user-images.githubusercontent.com/79555262/118375982-25d0a380-b57a-11eb-9fcf-43d64f29e783.png
</br>
<br>Here g is the gravitational constant, km and R are Armature resistance and torque constant respectively and can be obtained easily</br>
</p>
<p align="center"><br>
<img src=https://user-images.githubusercontent.com/79555262/118376007-43057200-b57a-11eb-92f0-b768bfc674e3.png
</br>
<br>The D matrix defined our d values for simplicities sake</br>
</p>


It can be determined the the two matrices A and B are controllable matrices. Thus a gain vector K can be found and a closed loop model (A - BK) can be found. The MATLab code for the creation of the model and controllibility determination can be found in Appendix A.

<div style="page-break-after: always;"></div>

# <b>Sensor Calibration:</b>

No sensor calibration is needed for this project.

<div style="page-break-after: always;"></div>

# <b>Controller Design and Simulations:</b>
Our system requires two controllers to be present in order to function properly. The first being a swing up control and the second being a balance control once the swing up has achieved its purpose.
<br>The swing up control will bring the pendulum from a resting position, swing it back and forth utilizing the angular velocity of the rotating inertial mass until the angle of the pendumlums bar is vertical (theta = pi). This will then allow the balance control to take over. </br>
Once the pendulum is vertical, the control will take over and maintain a theta close to pi in order to maintain a vertical position. This will last, still utilzing the inertial mass's angular velocity, until the system is turned off.
<br>
Once the linear model was created, the controllability was determined and the script was ran we obtained the linear simulation results
<p align="center"><br>
<img src=https://user-images.githubusercontent.com/79555262/118376543-6b42a000-b57d-11eb-97b3-8ca8cf08d379.png
</br>
<br>Output results of linear simulation of Inertial Wheel Pendulum</br>
</p>
<div style="page-break-after: always;"></div>

# <b>Controller Implementation</b>

The Inertial Wheel Pendulum can potetially be designed as a sort of metronome or timing clock. By turning off and on at repeated intervals, an oscillitory motion could be obtained. This, along with a sort of proximity sensor can be used to simulate a type of timing clock within a greater system.

<div style="page-break-after: always;"></div>

# <b>Appendix A - MATLab Code</b>
<br>Closed Loop Model:</br>
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
<br>J= (d11\*d22-d12\*d21)/d12;</br>
<br>D=[d11 d12;d21 d22];</br>
<br>Di=inv(D);</br>
<br>di11=Di(1,1)</br>
<br>di12=Di(1,2)</br>
<br>di21=Di(2,1)</br>
<br>di22=Di(2,2)</br>
<br>% Linear approximate model of IWP</br>
<br>A=[0 1 0;di11\*mbg 0 0;di21\*mbg 0 0]</br>
<br>B=[0;di12\*km/R;di22\*km/R]</br>
<br>% Controllability determination</br>
<br>disp('Is system controllable?');</br>
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
<br>Vp_=eig(A-B\*K)</br>

<br>% Close Loop Model</br>
<br>t = 0:0.01:5;</br>
<br>u = .001\*ones(size(t));</br>
<br>x0 = [0.01,0,0];</br>
<br>sys_cl = ss(A-B*K,B,C,0);</br>
<br>lsim(sys_cl,u,t,x0)</br>
<br>xlabel('Time (sec)')</br>
<br>ylabel('Pendulum Position (Theta)')</br>

<br>All Code obtained from [2]</br>

<div style="page-break-after: always;"></div>

# <b>References</b>
<br>[1] M. W. Spong, P. Corke, and R. Lozano, Nonlinear control of the reaction wheel pendulum,Automatica, vol. 37, no. 11, pp. 1845–1851, 2001</br>
<br>[2] V. M. Hernandez-Guzman and R. Silva-Ortigoza, Automatic Control with Experiments, Springer, 2019</br>
