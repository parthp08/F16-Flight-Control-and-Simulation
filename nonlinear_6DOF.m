function x_dot = nonlinear_6DOF(x, Fb, Mb)
% rigid body aircraft 6DOF nonlinear equations of motion
% in Body-axis with Flat Earth Assumption.
% x = [u,v,w,p,q,r,phi,theta,psi,xe,ye,h(=ze)), pdot]  ==> must be (13,1)
%%%% u = [delta_T, delta_e, delta_a, delta_r]

% u,v,w -> in m/s
% p,q,r -> in deg/sec
% phi,theta,psi -> deg
% xe,ye,h -> m
%%%% delta_T -> between 0 to 1
%%%% delta_e, delta_a, delta_r -> deg

% References : Ch. 7, Aircraft Flight Dynamics and Control,
%               by Wayne Durham.

global m Ib Ib_inv pdot;

% % debug
% disp(size(x));
% disp(size(Fb));
% disp(size(Mb));

% to speed up the calcultions
s_phi = sin(x(7));
c_phi = cos(x(7));
s_theta = sin(x(8));
c_theta = cos(x(8));
t_theta = tan(x(8));
s_psi = sin(x(9));
c_psi = cos(x(9));

% W-x matrix
omega_b = [0 -x(6) x(5); ...
         x(6) 0 -x(4); ...
         -x(5) x(4) 0];

% local horizontal(H) frame to body(B) frame
T_BH = [1 s_phi*t_theta c_phi*t_theta;
      0 c_phi -s_phi;
      0 s_phi/c_theta c_phi/c_theta];

% Body(B) frame to Earth-fixed(E) frame
T_EB = [c_theta*c_psi, -c_phi*s_psi + s_phi*s_theta*c_psi, ... 
                                  s_phi*s_psi + c_phi*s_theta*c_psi; ...
      c_theta*c_psi, c_phi*c_psi + s_phi*s_theta*s_psi, ...
                                  -s_phi*c_psi + c_phi*s_theta*s_psi; ...
      -s_theta, -s_phi*c_theta, -c_phi*c_theta;
];

%right now they are in "rad"  change to "deg" ???????????
%% Equations of Motion (Body axis)
x_dot = zeros(13,1);
x_dot(1:3,1) =  (Fb/m) - omega_b*x(1:3,1);
x_dot(4:6,1) = Ib_inv*(Mb - omega_b*Ib*x(4:6,1));
x_dot(7:9,1) =  T_BH*x(4:6,1);
x_dot(10:12,1) = T_EB*x(1:3,1);
x_dot(13) = pdot;
end
