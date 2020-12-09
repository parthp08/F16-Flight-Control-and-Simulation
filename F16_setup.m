% F16 Aircraft Simulation Setup file

clear all;
close all;
clc;

global m g Ib Ib_inv S c_bar b xcg_ref x_cg actual_power pdot;

% physical parameters
% Ref: Aircraft Control and Simulation, Stevens & Lewis.
g = 32.1740; %ft/s^2
m = 20500/32.1740; % lbs
S = 300; % wing area, ft^2
c_bar = 11.32; % mean aero cord, ft
b = 30; % wing span, ft
xcg_ref = 0.35;%*c_bar; % reference c.g. location
x_cg = 0.4;%*c_bar; % c.g. location % VARIABLE

RTOD = 57.2958;
DTOR = 1/RTOD;

% for engine
Hx = 160; % engine angular momentum, slug-ft2/sec, % IGNORING ITS EFFECTS
actual_power = 90; % actual engine power
pdot; % rate of change of engine power
% SHOUD NOT BE HERE % ??? -- parth

% inertia matrix    % slug*ft^2    
Ib = [9456, 0.0, 982;
    0.0, 55814, 0.0;
    982, 0.0, 63100];

% inverse inertia matrix
% precomputed for faster executions
Ib_inv =    1.0e-03 * [
    0.1059         0   -0.0016;
         0    0.0179         0;
   -0.0016         0    0.0159;
];

V_init = 500; % ft/s
alpha_init = 0.5; % rad
beta_init = -0.2; % rad
[u_init, v_init, w_init] = Valphabeta2uvw(V_init,alpha_init,beta_init);

% initial states
x_init = [
    u_init;     % u
    v_init;      % v 
    w_init;      % w
    0.7;      % p
    -0.8;      % q
    0.9;      % r
    -1;      % phi
    1;    % theta
    -1;      % psi
    1000;      % x
    900;      % y
    10000;  % h
    actual_power;
];

% control inputs
u = [ 
    0.9;    % throttle
    20*DTOR;    % elevator
    -15*DTOR;      % aileron
    -20*DTOR;      % rudder
];

FM_aero = F16_aerodynamics_old(x_init, u)
FM_prop = F16_propulsion(x_init, u);
FM_grav = gravity_model(x_init);

Fb = FM_aero(1:3,1) + FM_prop(1:3,1) + FM_grav(1:3,1);
Mb = FM_aero(4:6,1) + FM_prop(4:6,1);

xdot = nonlinear_6DOF(x_init, Fb, Mb)

[V_ans,alpha_ans,beta_ans] = ...
    Valphabeta_dot2uvw_dot(x_init(1),x_init(2),x_init(3), ...
    xdot(1),xdot(2),xdot(3))



% % Run simulink model
% TF = 60; % model run time, in seconds
% sim('six_dof.slx');
% 
% % Plot the outouts.
% t = ans.simX.Time;
% 
% u1 = ans.simU.Data(:,1);
% u2 = ans.simU.Data(:,2);
% u3 = ans.simU.Data(:,3);
% u4 = ans.simU.Data(:,4);
% 
% x1 = ans.simX.Data(:,1);
% x2 = ans.simX.Data(:,2);
% x3 = ans.simX.Data(:,3);
% x4 = ans.simX.Data(:,4);
% x5 = ans.simX.Data(:,5);
% x6 = ans.simX.Data(:,6);
% x7 = ans.simX.Data(:,7);
% x8 = ans.simX.Data(:,8);
% x9 = ans.simX.Data(:,9);
% x10 = ans.simX.Data(:,10);
% x11 = ans.simX.Data(:,11);
% x12 = ans.simX.Data(:,12);
% 
% % plot the control inputs
% figure
% subplot(4,1,1)
% plot(t,u1)
% legend("throttle")
% grid on
% subplot(4,1,2)V = 500;

% plot(t,u2)
% legend("elevator")
% grid on
% subplot(4,1,3)
% plot(t,u3)
% legend("aileron")
% grid on
% subplot(4,1,4)
% plot(t,u4)
% legend("rudder")
% grid on
% 
% % plot the states 
% figure
% subplot(3,4,1)
% plot(t,x1)
% legend("u")
% grid on
% subplot(3,4,2)
% plot(t,x2)
% legend("v")
% grid on
% subplot(3,4,3)
% plot(t,x3)
% legend("w")
% grid on
% subplot(3,4,4)
% plot(t,x4)
% legend("p")
% grid on
% subplot(3,4,5)
% plot(t,x5)
% legend("q")
% grid on
% subplot(3,4,6)
% plot(t,x6)
% legend("r")
% grid on
% subplot(3,4,7)
% plot(t,x7)
% legend("phi")
% grid on
% subplot(3,4,8)
% plot(t,x8)
% legend("theta")
% grid on
% subplot(3,4,9)
% plot(t,x9)
% legend("psi")
% grid on
% subplot(3,4,10)
% plot(t,x10)
% legend("x")
% grid on
% subplot(3,4,11)
% plot(t,x11)
% legend("y")
% grid on
% subplot(3,4,12)
% plot(t,x12)
% legend("h")
% grid on
% 
