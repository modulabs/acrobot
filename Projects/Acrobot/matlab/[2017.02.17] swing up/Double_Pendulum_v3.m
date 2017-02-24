%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverted pendulum - 2 links(rigid)
% Made by Gangnam Dynamics
% 2017. 02. 10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
clc;
close all;

%%
acrobot = AcrobotPlant;
acrobot.m1 = 10;    acrobot.m2 = 10;
acrobot.l1 = 10;    acrobot.l2 = 10;
acrobot.lc1 = acrobot.l1;   acrobot.lc2 = acrobot.l2;

%%
dt = 0.001;    %time step
t = 0:dt:30;   %time
n = length(t);



%% Initial Condition
q = deg2rad([-45; 30]);
dq = deg2rad([0; 0]);

alpha = 200;

des_ddq2 = 0;
des_dq2 = 0;
des_q2 = 0;   %radian

kp = 10;
kd = 10;

q_traj = q;   % save trajectory


hFigure = figure(2);

% plotAcrobot(hFigure, acrobot)
Ax = [0,0]; Ay = [0,0];
title('Double pendulum v2');
axis([-25 25 -25 25])
xlabel('length(m)')
ylabel('length(m)')

lower_link = animatedline;
lower_link.Color = 'magenta';
lower_link.LineWidth = 3;
lower_link.AlignVertexCenters = 'on';
lower_link.Marker = 'o';

upper_link = animatedline;
upper_link.Color = 'blue';
upper_link.LineWidth = 3;
upper_link.AlignVertexCenters = 'on';
upper_link.Marker = 'o';

grid on;

for i=1 : 1 : n-1 
    % dynamics
    [M, C] = acrobot.manipulatorDynamics(q, dq);  

    % control input
    M22_bar = M(2,2) - M(2,1)*inv(M(1,1))*M(1,2);
    C2_bar = C(2) - M(2,1)*inv(M(1,1))*C(1);
    
    des_q2 = 2*alpha/pi*atan(dq(1)*pi/180);
    v2 = des_ddq2 + kd*(des_dq2-dq(2)) + kp*(des_q2-q(2));
    v2 = 0;
    
    tau = [0; M22_bar*v2 + C2_bar];

    % forward dynamics
    ddq = inv(M) * (tau - C);

    %Euler method
    dq = dq + ddq*dt;
    q = q + dq*dt;
    
    % plot
    x1 = acrobot.l1*sin(q(1));
    y1 = -acrobot.l1*cos(q(1));

    x2 = x1 + acrobot.l2*sin(q(1) + q(2));
    y2 = y1 - acrobot.l2*cos(q(1) + q(2));

    % ========= LOWER LINK ==========    
    Ax = [0,x1];
    Ay = [0,y1];

    clearpoints(lower_link)
    addpoints(lower_link,Ax,Ay);

    % ========= UPPER LINK ==========  
    Ax = [x1,x2];
    Ay = [y1,y2];

    clearpoints(upper_link)
    addpoints(upper_link,Ax,Ay);

    drawnow
end

% figure(1);
% plot(t,q1*180/pi,t,q2*180/pi);
% legend('th1','th2');
% grid on

%% animation



for i = 1 : 5 : n

  
%     F(i) = getframe(hFigure);
end
% movie(F(i))







