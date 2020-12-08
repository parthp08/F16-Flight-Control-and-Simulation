function u_sat = control_saturation(u)
% saturate controls
% u = [delta_T, delta_e, delta_a, delta_r]

u_sat = zeros(4,1);

u_sat(1) = bound_function(u(1), 0, 1);
u_sat(2) = bound_function(u(2), -0.4363, 0.4363); % -25 to 25 deg
u_sat(3) = bound_function(u(3), -0.3752, 0.3752); % -21.5 to 21.5 deg
u_sat(4) = bound_function(u(4), -0.5236, 0.5236); % -30 to 30 deg

end