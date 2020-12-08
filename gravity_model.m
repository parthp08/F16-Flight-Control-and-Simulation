function FM_grav = gravity_model(x)
% Gravity forces and moments in body axis system
% 
% References : Ch. 7, Aircraft Flight Dynamics and Control,
%               by Wayne Durham.

global g;

F_grav = g*[sin(x(8)); sin(x(7))*cos(x(8)); cos(x(7))*cos(x(8))]; % N
M_grav = [0; 0; 0]; % N*m
FM_grav = [F_grav; M_grav];
end
