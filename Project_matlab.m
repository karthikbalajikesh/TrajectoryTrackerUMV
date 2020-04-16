%% *ALS Mid-Term Project*
% submitted by *: Karthik Balaji Keshavamurthi*
% *Problem 1. State Space Formulation:*
% 
% 
% 
% 
% *Equations of motion:*
% 
% To obtain the Dynamic equations of motion, we analyze all the forces and moments 
% on the submarine at respective directions and then use Newton's Second law to 
% obtain the following equations.
% 
% $\left(m+X_{\dot{u} } \right)\ddot{x} \;\;=\;-X_u \;\dot{x} \;\;\;+X_T \;\;$                                                        
% (1)
% 
% $\left(m+Z_{\dot{w} } \right)\ddot{z} \;\;=\;\;-Z_{\dot{q} } \;\theta^¨ \;\;-Z_w 
% \;\dot{z} \;+Z_q \;\theta^? \;+Z_s \;+Z_b$                            (2)
% 
% $\left(I_y \;+M_{\dot{q} } \right)\theta^¨ =\;\;-M_{\dot{w} } \;\ddot{z} -M_q 
% \;\theta^? \;-M_w \;\;\theta^? \;-x_{s\;} \;Z_s \;-x_b \;Z_b \;\;\;\;$              
% (3)
% 
% 
% 
% where the constants are:
% 
% $X_{\dot{u} }$ is the ?(-direction “added mass” from accelerating water (kg)
% 
% $X_u$ is the ?(-direction hydrodynamic “drag” coefficient (N•s/m)
% 
% $Z_{\dot{w} }$ is the ?(-direction “added mass” from accelerating water (kg)
% 
% $Z_w$ is the ?(-direction hydrodynamic “drag” coefficient (N•s/m)
% 
% $Z_{\dot{q} }$ is the ?(-direction “added mass” caused by rotation (kg•m)
% 
% $Z_q$ is the ?(-direction hydrodynamic drag caused by rotation (kg•m/s)
% 
% $M_{\dot{q} }$ is the “added rotational inertia” about the ?(-axis (kg•m2)
% 
% $M_q$ is the moment “drag” coefficient about the ?(-axis (N•m•s)
% 
% $M_{\dot{w} }$ is the “added rotational inertia” about the ?(-axis (kg•m)
% 
% $M_w$ is the moment “drag” coefficient about the ?(-axis (N•s)
% 
% $x_s$ is the position of the stern (rear) control surface in the ?(-direction 
% (m)
% 
% $x_b$ is the position of the bow (front) control surface in the ?(-direction 
% (m)
% 
% 
% 
% *Inputs:*
% 
% The inputs to the system are the following:
%% 
% * $X_{T\;}$- the thurst in the $x_0$ direction in newton.
% * $Z_s$ - the stern thrust in the $z_o$direction in newton
% * $Z_b$ - the blow thrust in the $z_0$direction in newton
%% 
% *Outputs:*
% 
% The outputs that we measure using our sensors are the following:
%% 
% * $\theta$ - The pitch angle in radians
% * z  - The depth in meters
% * x  - The position in meters
%% 
% *State Variables:*
% 
% To construct the state space equation, the following is my choice of state 
% variables.
% 
% $$\begin{array}{l}x_1 \;\;=\dot{x} \\x_{2\;} \;=x\\x_{3\;} \;=\dot{z} \\x_{4\;} 
% \;=z\\x_5 \;=\theta^? \\x_6 \;=\theta \end{array}$$
% 
% Using these state variables, we rearrange the Equations of motion to get the 
% state and output equations. 
% 
% *State Equations:*
% 
% $\dot{x_1 } \;=\left\lbrack \frac{-X_u }{\left(m+X_{\dot{u} } \right)}\right\rbrack 
% \;\ldotp x_1 \;\;\;+\left\lbrack \frac{1}{\left(m+X_{\dot{u} } \right)}\right\rbrack 
% \ldotp X_T$                                                                      
% (3)
% 
% $\dot{x_2 } ={\;\;x}_1$                                                                                                                        
% (4)
% 
% $\begin{array}{l}\dot{x_3 } =\left\lbrack \frac{\left(I_y \;Z_w \;+M_{\dot{q} 
% } \;Z_w -M_w \;Z_{\dot{q} } \right)}{\xi }\right\rbrack x_3 \;+\left\lbrack 
% \frac{\left(I_y {\;Z}_q +M_{\dot{q} } \;Z_q -M_q \;Z_{\dot{q} } \right)}{\xi 
% }\right\rbrack \;x_5 \;\\\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;+\left\lbrack 
% \frac{\left(-I_y -M_{\dot{q} } -Z_{\dot{q} } \;x_s \right)}{\xi }\right\rbrack 
% \;Z_s \;+\left\lbrack \frac{\left(-I_y -M_{\dot{q} } -Z_{\dot{q} } \;x_b \right)}{\xi 
% }\right\rbrack Z_b \end{array}$           (5)
% 
% $\dot{x_4 } =x_3$                                                                                                                         
% (6)
% 
% $\begin{array}{l}\dot{x_5 } =\left\lbrack \frac{-\left(m+Z_{\dot{w} } \right)\left(I_y 
% \;Z_w \;+M_{\dot{q} } \;Z_w -M_w \;Z_{\dot{q} } \right)\;-Z_w \xi }{Z_{\dot{q} 
% } \xi }\right\rbrack x_3 +\\\;\;\;\;\;\;\;\;\;\;\left\lbrack \frac{-\left(m+Z_{\dot{w} 
% } \right)\left(I_y \;Z_q \;+M_{\dot{q} } \;Z_q -M_q \;Z_{\dot{q} } \right)\;-Z_q 
% \xi }{Z_{\dot{q} } \xi }\right\rbrack x_5 \;\;+\\\;\;\;\;\;\;\;\;\;\left\lbrack 
% \frac{\left(I_y +M_{\dot{q} } +Z_{\dot{q} } \;x_s \right)\left(m+Z_{\dot{w} 
% } \right)+\xi }{Z_{\dot{q} } \xi }\right\rbrack Z_s +\left\lbrack \frac{\left(I_y 
% +M_{\dot{q} } +Z_{\dot{q} } \;x_b \right)\left(m+Z_{\dot{w} } \right)+\xi }{Z_{\dot{q} 
% } \xi }\right\rbrack Z_b \;\;\;\end{array}$         (7)
% 
% $\dot{x_6 } =x_5$                                                                                                                         
% (8)
% 
% where,
% 
% $$\xi \;=M_{\dot{w} } \;Z_{\dot{q} } -m\;I_y -m\;M_{\dot{q} } -Z_{\dot{w} 
% } I_y -\;M_{\dot{q} } Z_{\dot{w} }$$
% 
% 
% 
% *Output Equations:*
% 
% The output equations are:
% 
% $x=x_2$                                                                                                                          
% (9)
% 
% $z=x_4$                                                                                                                          
% (10)
% 
% $\theta =x_6$                                                                                                                          
% (11)
% 
% 
% 
% 

clear 
close all
clc
%% State Space Formulation


% I have used Alphabets to denote terms for simplicity during derivation of
% the State Space Equations.
syms A B C D E F G H J K XS XB XT ZS ZB M IY EPS
syms Xudot Xu Zwdot Zw Zqdot Zq Mqdot Mq Mwdot Mw Xs Xb XT Zs Zb m

EPS = J*E - M*IY - M*G - C*IY - G*C;
A1 = [(-B/(M+A)) 0 0 0 0 0
       1 0 0 0 0 0
       0 0 ((IY*D + G*D - K*E)/EPS) 0 ((IY*F + G*F - H*E)/EPS) 0
       0 0 1 0 0 0
       0 0 (-((M+C)/E)*((IY*D + G*D - K*E)/EPS) - (D/E)) 0 (-((M+C)/E)*((IY*F + G*F - H*E)/EPS) - (F/E)) 0
       0 0 0 0 1 0];
A1 = subs(A1,[A,B,C,D,E,F,G,H,J,K,M],[Xudot,Xu,Zwdot,Zw,Zqdot,Zq,Mqdot,Mq,Mwdot,Mw,m])
%% 
% 

   
B1 = [(1/(M+A)) 0 0
       0 0 0
       0 ((-IY - G - E*Xs)/EPS) ((-IY - G - E*Xb)/EPS)
       0 0 0
       0 (((IY+G+E*Xs)*(M+C) + EPS)/(EPS*E)) (((IY+G+E*Xb)*(M+C) + EPS)/(EPS*E))
       0 0 0];

B1 = subs(B1,[A,B,C,D,E,F,G,H,J,K,M],[Xudot,Xu,Zwdot,Zw,Zqdot,Zq,Mqdot,Mq,Mwdot,Mw,m])
C1 = [0 1 0 0 0 0 
      0 0 0 1 0 0
      0 0 0 0 0 1]
  D1 = zeros(3)
%% 
% 
%% Problem 2. Calculation of Parameters

% Let us update values for the constants
mass = 500;                           % in kg
L = 25;                            % in m
Iy = (1/300) * mass * (L^2);          % in kg m^2
X_udot = mass/30;                      % in kg
X_u = 94;                           % in N s/m
Z_wdot = (mass/10);                    % in kg
Z_w = 4.7e2;                        % in N s/m
Z_qdot = mass/20;                      % in kg m
Z_q = 9.5e2;                        % in kg m/s
M_qdot = Iy/20;                     % in kg m^2
M_q = 1.1e3;                        % in N m s
M_wdot = Iy/40;                     % in kg m 
M_w = 320;                          % in N s
xs = -L/3;                         % in m
xb = L/3;                          % in m
Umax = 3e6;                        % in N

%% Problem 3. State Space Representation of System:

A_matrix = double(subs(A1,[Xudot,Xu,Zwdot,Zw,Zqdot,Zq,Mqdot,Mq,Mwdot,Mw,m,IY],[X_udot,X_u,Z_wdot,Z_w,Z_qdot,Z_q,M_qdot,M_q,M_wdot,M_w,mass,Iy]))
B_matrix = double(subs(B1,[Xudot,Xu,Zwdot,Zw,Zqdot,Zq,Mqdot,Mq,Mwdot,Mw,m,IY,Xs,Xb],[X_udot,X_u,Z_wdot,Z_w,Z_qdot,Z_q,M_qdot,M_q,M_wdot,M_w,mass,Iy,xs,xb]))
C_matrix = C1
D_matrix = D1
%% Problem 4: Check for Minimum Realization:
% To check for this, we need to create an LTI object and then use the matlab 
% minreal function for checking for the minimum realisation.
% 
% From the Documentation, we can see that the minreal function filters out any 
% redundant states and provides the least realization. 
% 
% If the object returned due to this function is similar to the original LTI 
% object, our system is a minimum realization system.

% Let us first create the LTI Object
original_system = ss(A_matrix,B_matrix,C_matrix,D_matrix);
Minimum_Realisation = minreal(original_system)
%% 
% From above, you can see the State space matrices are the same as our original_system's 
% matrices. 
% 
% Hence, our State Space Model is a *Minimum Realization.*
% 
% 
%% Problem 5: Determine Controllability:
% We will have to find the Controllability matrix and check for its rank. I 
% have made a conditional statement for this purpose.

Q =  ctrb(A_matrix,B_matrix);

if rank(Q) == size(A_matrix,1)
    % This means that Matrix is controllable. If condition is satisfied,
    % the following statement would be the output
    disp('Matrix is Completely Controllable')  
else
    % If the rank is not full, the following statement will be the output.
    disp('Matrix is not Completely Controllable')
end
%% Problem 6: Compute Open Loop Poles:
% We will use the damp function to obtain the Natural Frequency, Damping Ratios 
% and Open loop poles.
% 
% Now, the Natual frequency values returned from this section will be in rad/s 
% and we are asked to display in terms of Hz. 
% 
% Hence, we will also convert them to the required format and use the matlab 
% Table function to show the results.

[natural_frequency, damping_ratios, open_loop_poles] =damp(original_system);
natural_frequency = natural_frequency * (1/(2*pi));
table(open_loop_poles,natural_frequency,damping_ratios,'VariableNames',{'Open Loop Poles', 'Natural Frequency in Hz', 'Damping Ratios'})
%% Problem 7: Initial Response:
% We are given initial a set of initial conditions. We will use the matlab function 
% 'inital' to get the response of the function in open loop and plot the response 
% in a subplot. 

initial_condition_vector = [100,-415,-300,200,0,45*pi/180]';
[open_loop_output, time, open_loop_state] = initial(original_system,initial_condition_vector);
%% 
% Now we also have to find the settling time and pplace a marker on the response. 
% 
% For this we will first use the lsiminfo command to find the settling time 
% and then place a marker. 

% The Default Settling Time for lsiminfo is 2%. To change it to 5%, we will
% need to add two extra arguments to lsiminfo other than output and time.
s = lsiminfo(open_loop_output,time,'SettlingTimeThreshold',0.05)
% Now we have the three settling times. To place a marker, we need to find
% the index of that specific time. 

[~,index_output1] = min(abs(time - s(1).SettlingTime));
[~,index_output2] = min(abs(time - s(2).SettlingTime));
[~,index_output3] = min(abs(time - s(3).SettlingTime));

% Now we plot the graphs. The x and z have the same units and hence will be
% plot in the same graph. 
subplot(2,1,1)
title('Position vs Time')
plot(time, open_loop_output(:,1:2),'linewidth',1.5)
hold on
plot(time(index_output1,1),open_loop_output(index_output1,1),'r*')
hold on 
plot(time(index_output2,1),open_loop_output(index_output2,2),'k*')
xlabel('time in seconds')
ylabel('Position in m')
legend('Position response in x', 'Position response in z','x at 5% Settling time','z at 5% Settling Time')

subplot(2,1,2)
plot(time, open_loop_output(:,3),'linewidth',1.5)
hold on
plot(time(index_output3,1),open_loop_output(index_output3,3),'k*')
title('Pitch Angle \theta vs Time')
legend('\theta response','\theta at 5% settling time')
xlabel('time in seconds')
ylabel('\theta in radians')
%% Problem 8 - Full State Feedback Controller
% Now we will start designing the full state feedback controller.
G = place(A_matrix, B_matrix, [-3.5,-4,-4,-3,-3.5,-3.7]);
%G = place(A_matrix, B_matrix, [-3.5,-4,-4,-3,-3.5,-3.7]);
%G = place(A_matrix, B_matrix, [-3,-4.5,-4,-5,-5.5,-6]);
A1 = A_matrix-(B_matrix*G);
%C_new = 
new_sys = ss(A1,B_matrix,C_matrix,D_matrix);
[y1, t1, x1] = initial(new_sys,initial_condition_vector);
%%
pos_x = y1(:,1)+465;
pos_z = y1(:,2)+150;
theta = y1(:,3);
animate_auv(t1,pos_x,pos_z,theta)