function [F_prop, M_prop] = F16_propulsion(x,u_sat)
% Calculate Propulsive Forces and Moments 
% u_sat = [delta_T, delta_e, delta_a, delta_r]
%
% based on work for Stanle Bak
% https://github.com/stanleybak/AeroBenchVVPython

global pdot;

%% TGEAR function
% convert command [0,1] to commanded power [0,100]
if (u_sat(1) <= .77)
    comm_power = 64.94 * u_sat(1);  % commanded power
else
    comm_power = 217.38 * u_sat(1) - 117.38;
end

%% RTAU function
% time constant for first order lag in engine power dynamic repsonse
    function rt = rtau(delta_power)
        if(delta_power <= 25)
            rt = 1.0;
        elseif(delta_power >= 50)
            rt = .1;
        else
            rt = 1.9 - .036*delta_power;
        end
    end
    
%% PDOT function
% comptute rate of change power fromm given actual power and commanded
% power
if(x(13) >= 50)
    if(comm_power >= 50)
        t = 5;
        p2 = x(13);
    else
        p2 = 60;
        t = rtau(p2 - comm_power);
    end
else
    if (comm_power >= 50)
        t = 5;
        p2 = 40;
    else
        p2 = x(13);
        t = rtau(p2 - comm_power);
    end
end

pdot = t * (p2 - comm_power);

%% THRUST function
% calculate engine thrust
[~,a] = atmospheric_model(x(12)); % speed of sound at current x(12)itude
V = sqrt(x(1)^2 + x(2)^2 + x(3)^2);
M = V/a; % mach number

a = [1060,670,880,1140,1500,1860;...
    635,425,690,1010,1330,1700;...
    60,25,345,755,1130,1525;...
    -1020,-170,-300,350,910,1360;...
    -2700,-1900,-1300,-247,600,1100;...
    -3600,-1400,-595,-342,-200,700];

b = [12680,9150,6200,3950,2450,1400;...
    12680,9150,6313,4040,2470,1400;...
    12610,9312,6610,4290,2600,1560;...
    12640,9839,7090,4660,2840,1660;...
    12390,10176,7750,5320,3250,1930;...
    11680,9848,8050,6100,3800,2310;];

c = [20000,15000,10800,7000,4000,2500;...
    21420,15700,11225,7323,4435,2600;...
    22700,16860,12250,8154,5000,2835;...
    24240,18910,13760,9285,5700,3215;...
    26070,21075,15975,11115,6860,3950;...
    28886,23319,18300,13484,8642,5057];

if (x(12)<0)
    x(12) = 0.01;
end

h =.0001 * x(12);
i = fix(h);  
if(i >= 5)
    i = 4;
end

dh = h - i;
rm = 5 * M;
m = fix(rm);
if(m >= 5)
    m = 4;
end

dm = rm - m;
cdh = 1 - dh;

i = i + 1;
m = m + 1;

s = b(i, m) * cdh + b(i + 1, m) * dh;
t = b(i, m + 1) * cdh + b(i + 1, m + 1) * dh;
tmil = s + (t - s) * dm;

if (x(13) < 50)
    s = a(i, m) * cdh + a(i + 1, m) * dh;
    t = a(i, m + 1) * cdh + a(i + 1, m + 1) * dh;
    tidl = s + (t - s) * dm;
    thrust = tidl + (tmil - tidl) * x(13) * .02;
else 
    s = c(i, m) * cdh + c(i + 1, m) * dh;
    t = c(i, m + 1) * cdh + c(i + 1, m + 1) * dh;
    tmax = s + (t - s) * dm;
    thrust = tmil + (tmax - tmil) * (x(13) - 50) * .02;
end


%% Propulsive Force and Moment about body axis
F_prop = [thrust; 0; 0];
M_prop = [0; 0; 0]; % N*m1
end