function [rho, a] = atmospheric_model(altitude)
% calculate the atmospheric density(in Kg/m^3) and speed of sound
% at the given altitude
% 
% Ref: U.S. Standard Atmosphere, 1976

global g;
R_air = 287.05287;  % Specific gas constant(for dry air), J/Kg·K

if (0 <= altitude) && (altitude < 11000)
    alpha = -0.0065;  % K/m
    T0 = 288.15;      % K
    p0 = 101325.0;    % Pa
    
    T = T0 + alpha * altitude;
    p = p0 * (T0 / T) ^ (g / (R_air * alpha));
    
elseif (11000 <= altitude) && (altitude < 20000)
    T = 216.65;    % K
    p0 = 22632.1;  % Pa
    h0 = 11000;    % m

    p = p0 * exp(-g * (altitude - h0) / (R_air * T));

elseif (20000 <= altitude) && (altitude < 32000)
    alpha = 0.001;    % K/m
    T0 = 216.65;      % K
    p0 = 5474.89;     % Pa
    h0 = 20000;       % m

    T = T0 + alpha * (altitude - h0);
    p = p0 * (T0 / T) ^ (g / (R_air * alpha));

elseif (32000 <= altitude) && (altitude < 47000)
    alpha = 0.0028;   % K/m
    T0 = 228.65;      % K
    p0 = 868.019;     % Pa
    h0 = 32000;       % m

    T = T0 + alpha * (altitude - h0);
    p = p0 * (T0 / T) ^ (g / (R_air * alpha));

elseif (47000 <= altitude) && (altitude < 51000)
    T = 270.65;    % K
    p0 = 110.906;  % Pa
    h0 = 47000;    % m

    p = p0 * exp(-g * (altitude - h0) / (R_air * T));

elseif (51000 <= altitude) && (altitude < 71000)
    alpha = -0.0028;  % K/m
    T0 = 270.65;      % K
    p0 = 66.9389;     % Pa
    h0 = 51000;       % m

    T = T0 + alpha * (altitude - h0);
    p = p0 * (T0 / T) ^ (g / (R_air * alpha));

elseif (71000 <= altitude) && (altitude < 84500)
    alpha = -0.002;   % K/m
    T0 = 214.65;      % K
    p0 = 3.95642;     % Pa
    h0 = 71000;       % m

    T = T0 + alpha * (altitude - h0);
    p = p0 * (T0 / T) ^ (g / (R_air * alpha));

else
    error("altitude must be below 84500 m");
end

rho = p / (R_air * T);

% a = sqrt(mu, R, T).
% mu = ratio of specific heats (1.4 for air)
% R = gas const (286 m^2/s^2/K for air)
a = sqrt(1.4*286*T);
end