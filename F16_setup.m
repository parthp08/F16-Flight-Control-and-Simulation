% F16 Aircraft Parameters   
% Global Variables
% Ref: Aircraft Control and Simulation, Stevens & Lewis.

clear all;
close all;
clc;

global m g Ib Ib_inv S c_bar b xcg_ref x_cg actual_power pdot;

%% physical parameters
m = 9298.6436; % kg
g = 9.81;   % m/s^2
S = 27.8709; % wing area, m^2
c_bar = 3.450336; % mean aero cord, m
b = 9.144; % wing span, m
xcg_ref = 0.35*c_bar; % reference c.g. location
x_cg = 0.35*c_bar; % c.g. location % VARIABLE

%% for engine
Hx = 216.931; % engine angular momentum, kg/m^2, % IGNORING ITS EFFECTS
actual_power = 90; % actual engine power
pdot; % rate of change of engine power
% SHOUD NOT BE HERE % ??? -- parth

% inertia matrix    % kg*m^2    
Ib = [12874.85, 0.0, 1331.41;
    0.0, 75673.62, 0.0;
    1331.41, 0.0, 85552.11];

%% inverse inertia matrix
% precomputed for faster executions
Ib_inv = 1e-4 * [
    0.7780         0   -0.0121;
         0    0.1321         0;
   -0.0121         0    0.1171;
];


% initial states


% control saturation


