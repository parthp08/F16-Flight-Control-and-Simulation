% 3DOF Linear Longitudinal Dynamics

% Trimmed Condition: (Steady, Level, Symmetric Flight) 

% matrices are obtained from "Model Linearizer" Tool in Simulink

                %   V   alpha  q  theta  Xe  h     power
%      x_initial = [500,  0,   0,  0,    0, 25000, 4.66];
%      u_initial = [0.5, -3.0159, 0.25]; % deg


clear all;
clc;

%   V             alpha          q           theta        h
A = [
-0.008421         6.934        0.3438      -0.05048     7.688e-05       % V
-2.909e-05       -0.5293        0.9565             0     5.099e-07      % alpha
-2.655e-11        -2.998       -0.7552             0     2.344e-13      % q
         0             0             1             0             0      % theta 
-5.168e-19          -500             0           500             0      % h
]; 

  B = [
0.02985;        % V
-0.0009551;     % alpha
-0.09189;       % q
0;              % theta
0;              % h
];

D = [0];

s = tf('s');

% TRANSFER FUNCTION MODELS -- For above mention linear condition

% q to elevator transfer function
C = [0, 0, 1*rad2deg(1), 0, 0];
[num_,den_] = ss2tf(A,B,C,D);
q_to_elevator = zpk(tf(num_,den_))

% theta to elevator transfer function
C = [0, 0, 0, 1*rad2deg(1), 0];
[num_,den_] = ss2tf(A,B,C,D);
theta_to_elevator = zpk(tf(num_,den_))

% h to elevator transfer function
C = [0, 0, 0, 0, 1];
[num_,den_] = ss2tf(A,B,C,D);
h_to_elevator = zpk(tf(num_,den_))

% V to elevator transfer function
C = [1, 0, 0, 0, 0];
[num_,den_] = ss2tf(A,B,C,D);
V_to_elevator = zpk(tf(num_,den_));
% lead-lag compensator for V to elevator transfer function
lead_lag = -4.202*(s+0.004341)/(0.004341*(s-4.202));
V_to_elevator_with_lead_lag = lead_lag*V_to_elevator;
V_to_elevator_with_lead_lag = minreal(V_to_elevator_with_lead_lag, 0.001)






