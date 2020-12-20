close all
%% Reference
% Alt0=15000;
% AltTarget=10;
% Sigma=.01;
% THRTL=5000;
% Define State-space functions
A11=(Xu+Tu)/m; A12=Xw/m;
A21=Zu/m;A22=Zw/m;A23=(Zq+m*Vref)/m;
A31=Mu/Iyy;A32=Mw/Iyy;A33=Mq/Iyy;
A=[A11 A12 0 -g;A21 A22 A23 0;A31 A32 A33 0;0 0 1 0];
B11=TdT/m; B12=Xdm/m;
B22=Zdm/m;
B32=Mdm/Iyy;
B=[B11 B12; 0 B22; 0 B32; 0 0];
C=A;
D=zeros(5,2);
%% Numerically integrate ODE.
SimOut = sim('LinearizedF16.mdl','AbsTol','1e-6','RelTol','1e-6',...
        'SaveState','on','StateSaveName','Xout',...
        'SaveOutput','on','OutputSaveName','Yout');
    
SimOutVars = SimOut.who;
Outputport = SimOut.get('Yout');


t=Outputport{1}.Values.Data; % s, time
Alt=Outputport{2}.Values.Data; % unitless, reference
y=Outputport{3}.Values.Data; % unitless, Measurement
e=Outputport{4}.Values.Data; % unitless, Error
u=Outputport{5}.Values.Data; % unitless, Controller output
r=Outputport{6}.Values.Data;

%Post-processing
% Out(1,:)=t;
% Out(2,:)=Alt;
% Out(3,:)=y;
% Out(4,:)=e;
% Out(5,:)=u;
% %Out(6,:)=w2;
% Out=Out';


% Rotary flexible-joint testbed parameters
% Emmanuel Vergara
% September 26, 2018

font_size = 15;
line_size = 15;
line_width = 2;

%------------------------------%figure 1-----------------------------------------------------

figure
plot(t,Alt,t,r,'Linewidth',line_width);
title('Reference and Measurement vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('Reference and Measurement','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
legend('Reference','Measurement')
% print -depsc -r720 plots/speed_vs_time
%exportfig(gcf,'plots/speed_vs_time.eps','width',11,'Height',8.5,'fontmode','fixed','fontsize',18,'Color','cmyk','LineWidth',line_width);

%------------------------------%figure 2-----------------------------------------------------

figure
plot(t,e,'Linewidth',line_width);
title('Error vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('Error','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
%legend('Reference','Measurement')
% print -depsc -r720 plots/speed_vs_time
%exportfig(gcf,'plots/speed_vs_time.eps','width',11,'Height',8.5,'fontmode','fixed','fontsize',18,'Color','cmyk','LineWidth',line_width);

%------------------------------%figure 3-----------------------------------------------------

figure
plot(t,u,'Linewidth',line_width);
title('Controller output vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('Controller output','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
grid on
%legend('Reference','Measurement')
% print -depsc -r720 plots/speed_vs_time
%exportfig(gcf,'plots/speed_vs_time.eps','width',11,'Height',8.5,'fontmode','fixed','fontsize',18,'Color','cmyk','LineWidth',line_width);
