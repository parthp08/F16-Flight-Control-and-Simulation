function FM_aero = F16_aerodynamics(x,u_sat)
% Compute F16 Body-axis Aerodynamics forces and moments
% u_sat = [delta_T, delta_e, delta_a, delta_r]
%
% Arodynamic model for F16 in form of Polynomial function
% is used here instead of data tables as described in [1]
%
% Ref:
% [1] GLOBAL NONLINEAR PARAMETRIC MODELING WITH APPLICATION TO 
%     F-16 AERODYNAMICS, by Eugene A. Morelli.

global S xcg_ref x_cg b c_bar;

% convert body-axis velocity to (alpha,beta,V)
V = sqrt(x(1)^2 + x(2)^2 + x(3)^2); % m/sec
alpha = atan(x(3)/x(1)); % rad
beta = asin(x(2)/V); % rad

% variable ranges  % bound
%    -- Table 1 [1]
alpha = bound_function(alpha,-0.1745,0.7854); % -10 to 45 deg
beta = bound_function(beta,-0.5236,0.5236); % -30 to 30 deg

% aerodynamic polynomial model
%    -- Table 2 [1]
Cx_0 = -1.943367e-02 + 2.136104e-01*alpha - 2.903457e-01*(u_sat(2)^2) ...
       -3.348641e-03*u_sat(2) - 2.060504e-01*alpha*u_sat(2) + ...
       6.988016e-01*(alpha^2) - 9.035381e-01*(alpha^3);
Cx_q = 4.833383e-01 + 8.644627e+00*alpha + 1.131098e+01*(alpha^2) ...
       - 7.422961e+01*(alpha^3) + 6.075776e+01*(alpha^4);
Cy_0 = -1.145916e+00*beta + 6.016057e-02*u_sat(3) + 1.642479e-01*u_sat(4);
Cy_p = -1.006733e-01 + 8.679799e-01*alpha + 4.260586e+00*(alpha^2) ...
       - 6.923267e+00*(alpha^3);
Cy_r = 8.071648e-01 + 1.189633e-01*alpha + 4.177702e+00*(alpha^2) ...
       - 9.162236e+00*(alpha^3);
Cz_0 = (1-(beta^2))*(-1.378278e-01 - 4.211369e+00*alpha + ...
        4.775187e+00*(alpha^2) - 1.026225e+01*(alpha^3) + ...
        8.399763e+00*(alpha^4)) - 4.354000e-01*u_sat(2);
Cz_q = -3.054956e+01 - 4.132305e+01*alpha + 3.292788e+02*(alpha^2) ...
       - 6.848038e+02*(alpha^3) + 4.080244e+02*(alpha^4);
Cl_0 = -1.058583e-01*beta - 5.776677e-01*alpha*beta - ...
       1.672435e-02*(alpha^2)*beta + 1.357256e-01*(beta^2) ...
       + 2.172952e-01*alpha*(beta^2) + 3.464156e+00*(alpha^3)*beta ...
       - 2.835451e+00*(alpha^4)*beta - 1.098104e+00*(alpha^2)*(beta^2);
Cl_p = -4.126806e-01 - 1.189974e-01*alpha + 1.247721e+00*(alpha^2) ...
       - 7.391132e-01*(alpha^3);
Cl_r = 6.250437e-02 + 6.067723e-01*alpha - 1.101964e+00*(alpha^2) ...
       + 9.100087e+00*(alpha^3) - 1.192672e+01*(alpha^4);
Cl_delta_a = -1.463144e-01 - 4.073901e-02*alpha + 3.253159e-02*beta ...
             + 4.851209e-01*(alpha^2) + 2.978850e-01*alpha*beta ...
             - 3.746393e-01*(alpha^2)*beta - 3.213068e-01*(alpha^3);
Cl_delta_r = 2.635729e-2 - 2.192910e-02*alpha - 3.152901e-03*beta ...
             - 5.817803e-02*alpha*beta + 4.516159e-01*(alpha^2)*beta ...
             - 4.928702e-01*(alpha^3)*beta - 1.579864e-02*(beta^2);
Cm_0 = -2.029370e-02 + 4.660702e-02*alpha - 6.012308e-01*u_sat(2) ...
       -8.062977e-02*alpha*u_sat(2) + 8.320429e-02*(u_sat(2)^2) + ...
       5.018538e-01*(alpha^2)*u_sat(2) + 6.378864e-01*(u_sat(2)^3) ...
       + 4.226356e-01*alpha*(u_sat(2)^2);
Cm_q = -5.159153e+00 - 3.554716e+00*alpha - 3.598636e+01*(alpha^2) + ...
       2.247355e+02*(alpha^3) - 4.120991e+02*(alpha^4) ...
       + 2.411750e+02*(alpha^5);
Cn_0 = 2.993363e-01*beta + 6.594004e-02*alpha*beta - ...
       2.003125e-01*(beta^2) - 6.233977e-02*alpha*(beta^2) ...
       - 2.107885e+00*(alpha^2)*beta + 2.141420e+00*(alpha^2)*(beta^2) ...
       + 8.476901e-01*(alpha^3)*beta;
Cn_p = 2.677652e-02 - 3.298246e-01*alpha + 1.926178e-01*(alpha^2) ...
       + 4.013325e+00*(alpha^3) - 4.404302e+00*(alpha^4);
Cn_r = -3.698756e-01 - 1.167551e-01*alpha - 7.641297e-01*(alpha^2);
Cn_delta_a = -3.348717e-02 + 4.276655e-02*alpha + 6.573646e-03*beta ... 
             + 3.535831e-01*alpha*beta - 1.373308e+00*(alpha^2)*beta ...
             + 1.237582e+00*(alpha^3)*beta + 2.302543e-01*(alpha^2) ...
             - 2.512876e-01*(alpha^3) + 1.588105e-01*(beta^3) ...
             - 5.199526e-01*alpha*(beta^3);
Cn_delta_r = -8.115894e-02 - 1.156580e-02*alpha + 2.514167e-02*beta ...
             + 2.038748e-01*alpha*beta - 3.337476e-01*(alpha^2)*beta ... 
             + 1.004297e-01*(alpha^2);
   
% dynamic pressure
[rho,~] = atmospheric_model(x(12));
q_bar = 0.5*rho*V*V;

% non-dimensional
p_nd = 0.5*x(4)*b/V;
q_nd = 0.5*x(5)*c_bar/V;
r_nd = 0.5*x(6)*b/V;

% aerodynamic forces
Cx = Cx_0 + q_nd*Cx_q;
Cy = Cy_0 + p_nd*Cy_p + r_nd*Cy_r;
Cz = Cz_0 + q_nd*Cz_q;

% aerodynamic moments
Cl = Cl_0 + p_nd*Cl_p + r_nd*Cl_r + u_sat(3)*Cl_delta_a + ...
        u_sat(4)*Cl_delta_r;
Cm = Cm_0 + q_nd*Cm_q + Cz*(xcg_ref - x_cg);
Cn = Cn_0 + p_nd*Cn_p + r_nd*Cn_r + u_sat(3)*Cn_delta_a + ...
        u_sat(4)*Cn_delta_r - Cy*c_bar*(xcg_ref - x_cg)/b;
    
F_aero = q_bar*S*[Cx; Cy; Cz];
M_aero = q_bar*S*[Cl*b; Cm*c_bar; Cn*b];

FM_aero = [F_aero; M_aero];
end