% quick test to see if whole model is running correctly
% ref values are taken from stevens 2nd edition and
% https://github.com/stanleybak/AeroBenchVVPython

clear all;
close all;
clc;

global m g Ib Ib_inv S c_bar b xcg_ref x_cg actual_power pdot;

% physical parameters
% Ref: Aircraft Control and Simulation, Stevens & Lewis.
g = 32.17; %ft/s^2
m = 20500/32.17; % lbs
S = 300; % wing area, ft^2
c_bar = 11.32; % mean aero cord, ft
b = 30; % wing span, ft
xcg_ref = 0.35;%*c_bar; % reference c.g. location
x_cg = 0.4;%*c_bar; % c.g. location % VARIABLE

RTOD = 57.29578;
DTOR = 1/RTOD;

% for engine
Hx = 160; % engine angular momentum, slug-ft2/sec, % IGNORING ITS EFFECTS
actual_power = 90; % actual engine power
pdot; % rate of change of engine power

% inertia matrix    % slug*ft^2    
Ib = [9496, 0.0, -982;
    0.0, 55814, 0.0;
    -982, 0.0, 63100];

% inverse inertia matrix
% precomputed for faster executions
Ib_inv =    1.0e-03 * [
    0.1055         0    0.0016;
         0    0.0179         0;
    0.0016         0    0.0159;
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

FM_aero = F16_aerodynamics_old(x_init, u);
FM_prop = F16_propulsion(x_init, u);
FM_grav = gravity_model(x_init);

Fb = FM_aero(1:3,1) + FM_prop(1:3,1) + FM_grav(1:3,1);
Mb = FM_aero(4:6,1) + FM_prop(4:6,1);

xdot = nonlinear_6DOF(x_init, Fb, Mb);

[Vdot_ans,alphadot_ans,betadot_ans] = ...
    uvw_dot2Valphabeta_dot(x_init(1),x_init(2),x_init(3), ...
    xdot(1),xdot(2),xdot(3));

%% TESTING
% Ref: stevens, 2nd Ed. p.187
xdot_ref = [102.35634804255098;
    -217.7680815087604;
    -433.16808306205763;
    12.833225545333605;
    0.9048400786421307;
    1.8769265762733753;
    2.5057346157773357;
    0.3250820416325951;
    2.145926179723922;
    342.44390305237005;
    -266.77068149494636;
    248.12411562961984; 
    -58.69
    ];

FM_aero_ref = [3765.9530491106048;
    12390.512477907021;
    -107093.15448228816;
    115300.4954085512;
    16415.138317083183;
    79202.97182183582;
    ];

Thrust_ref = 15912.06494550379;

FM_grav_ref = [-27.070121581270033;
    -14.62604911049109;
    9.391278134139215;
    0; 0; 0;
    ];

if max(abs(FM_aero - FM_aero_ref)) >= 10
    disp("AERODYNAMIC TEST FAILURE");
else
    disp("AERODYNAMIC TEST SUCCESS");
end

if max(abs(FM_prop(1,1) - Thrust_ref)) >= 10
    disp("PROPULSION TEST FAILURE");
else
    disp("PROPULSION TEST SUCCESS");
end

if max(abs(FM_grav - FM_grav_ref)) >= 0.1
    disp("GRAVITY TEST FAILURE");
else
    disp("GRAVITY TEST SUCCESS");
end

if (max(max(abs(xdot(1:3) - xdot_ref(1:3))) >= 30) && ...
        (max(abs(xdot(4:13) - xdot_ref(4:13))) >= 0.01))
    disp("MODEL TEST FAILURE");
else
    disp("MODEL TEST SUCCESS");
end
