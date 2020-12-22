function [rho, a] = atmospheric_model(altitude)
% calculate the atmospheric density(in Kg/m^3) and speed of sound
% at the given altitude
% 
% Ref: Stevens & Lewis, "Aircraft Control and Simulation"

    ro = 2.377e-3; % Slug/ft^3
    tfac = 1 - .703e-5 * altitude;

    if altitude >= 35000 % in stratosphere
        t = 390;
    else
        t = 519 * tfac; % 3 rankine per atmosphere (3 rankine per 1000 ft)
    end
    rho = ro * tfac^4.14;

    %speed of sound
    a = sqrt(1.4 * 1716.3 * t);
end