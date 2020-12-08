% F16 Aircraft Parameters   
% Global Variables
% Ref: Aircraft Control and Simulation, Stevens & Lewis.

clear all;
close all;
clc;

global m g Ib Ib_inv S c_bar b xcg_ref x_cg actual_power pdot;

disp('Hi, Parth');

% physical parameters
m = 9298.6436; % kg
g = 9.81;   % m/s^2
S = 27.8709; % wing area, m^2
c_bar = 3.450336; % mean aero cord, m
b = 9.144; % wing span, m
xcg_ref = 0.35*c_bar; % reference c.g. location
x_cg = 0.35*c_bar; % c.g. location % VARIABLE

% for engine
Hx = 216.931; % engine angular momentum, kg/m^2, % IGNORING ITS EFFECTS
actual_power = 90; % actual engine power
pdot; % rate of change of engine power
% SHOUD NOT BE HERE % ??? -- parth

% inertia matrix    % kg*m^2    
Ib = [12874.85, 0.0, 1331.41;
    0.0, 75673.62, 0.0;
    1331.41, 0.0, 85552.11];

% inverse inertia matrix
% precomputed for faster executions
Ib_inv = 1e-4 * [
    0.7780         0   -0.0121;
         0    0.1321         0;
   -0.0121         0    0.1171;
];


% initial states
x_init = [
    85;     % u
    0;      % v 
    0;      % w
    0;      % p
    0;      % q
    0;      % r
    0;      % phi
    0;    % theta
    0;      % psi
    0;      % x
    0;      % y
    10000;  % h
    actual_power;
];

% control inputs
u = [ 
    0.2;    % throttle
    0;    % elevator
    0;      % aileron   
    0;      % rudder
];

% Run simulink model
TF = 60; % model run time, in seconds
sim('six_dof.slx');

% Plot the outouts.
t = ans.simX.Time;

u1 = ans.simU.Data(:,1);
u2 = ans.simU.Data(:,2);
u3 = ans.simU.Data(:,3);
u4 = ans.simU.Data(:,4);

x1 = ans.simX.Data(:,1);
x2 = ans.simX.Data(:,2);
x3 = ans.simX.Data(:,3);
x4 = ans.simX.Data(:,4);
x5 = ans.simX.Data(:,5);
x6 = ans.simX.Data(:,6);
x7 = ans.simX.Data(:,7);
x8 = ans.simX.Data(:,8);
x9 = ans.simX.Data(:,9);
x10 = ans.simX.Data(:,10);
x11 = ans.simX.Data(:,11);
x12 = ans.simX.Data(:,12);

% plot the control inputs
figure
subplot(4,1,1)
plot(t,u1)
legend("throttle")
grid on
subplot(4,1,2)
plot(t,u2)
legend("elevator")
grid on
subplot(4,1,3)
plot(t,u3)
legend("aileron")
grid on
subplot(4,1,4)
plot(t,u4)
legend("rudder")
grid on

% plot the states 
figure
subplot(3,4,1)
plot(t,x1)
legend("u")
grid on
subplot(3,4,2)
plot(t,x2)
legend("v")
grid on
subplot(3,4,3)
plot(t,x3)
legend("w")
grid on
subplot(3,4,4)
plot(t,x4)
legend("p")
grid on
subplot(3,4,5)
plot(t,x5)
legend("q")
grid on
subplot(3,4,6)
plot(t,x6)
legend("r")
grid on
subplot(3,4,7)
plot(t,x7)
legend("phi")
grid on
subplot(3,4,8)
plot(t,x8)
legend("theta")
grid on
subplot(3,4,9)
plot(t,x9)
legend("psi")
grid on
subplot(3,4,10)
plot(t,x10)
legend("x")
grid on
subplot(3,4,11)
plot(t,x11)
legend("y")
grid on
subplot(3,4,12)
plot(t,x12)
legend("h")
grid on

