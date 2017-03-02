%% 
clear all;
clc;
close all;


%% model parameter
acrobot = AcrobotPlant; % drake class
acrobot.l1 = 0.2; 
acrobot.l2 = 0.2;  
acrobot.m1 = 1.9008; 
acrobot.m2 = 0.7175;  
acrobot.lc1 = 1.8522*10^-1; 
acrobot.lc2 = 6.2052*10^-2;
acrobot.Ic1 = 4.3399*10^-3;
acrobot.Ic2 = 5.2285*10^-3;

%% initialization
FinalTime = 30;         % sec
dt = 0.001;             % time step (sec)
t = 0:dt:FinalTime;     % time
n = length(t);

% control = 'passive';
control = 'collocated pfl';
% control = 'lqr';


if strcmp(control, 'passive')
    q = deg2rad([45; 0]);
    q_dot = deg2rad([10; 10]);
elseif strcmp(control, 'collocated pfl')
    q = deg2rad([45; 0]);
    q_dot = deg2rad([0; 0]);
    
    des_q2_ddot = 0;
    des_q2_dot = 0;
    des_q2 = 0;   %radian
    
    alpha = 22;
    kp = 50;
    kd = 50;
elseif strcmp(control, 'lqr')
    q = [pi + deg2rad(2); -deg2rad(2)]; %q = [pi+0.04;-0.05];
    q_dot = [deg2rad(-11); 0]; % q_dot = [-0.2;0.04];
    q_ddot = [0;0];
    K = [-113.8908, -9.5070, -17.3951, -1.9405];
    y = zeros(4,1);
    y_d = [pi;0;0;0];
end

%%

for i=1 : 1 : n-1
    [H,C,B] = acrobot.manipulatorDynamics(q,q_dot);
    
    if strcmp(control, 'passive')
        u = [0; 0];
    elseif strcmp(control, 'collocated pfl')
        H22_bar = H(2,2) - H(2,1)*inv(H(1,1))*H(1,2);
        C2_bar = C(2) - H(2,1)*inv(H(1,1))*C(1);
        
        des_q2 = 2*alpha/pi*atan(q_dot(1)); %*pi/180);
        v2 = des_q2_ddot + kd*(des_q2_dot - q_dot(2)) + kp*(des_q2 - q(2));

        u = [0; H22_bar*v2 + C2_bar];
    elseif strcmp(control,'noncollocated pfl')
        temp = 1;
    elseif strcmp(control, 'lqr')
        y = [q(1);q(2);q_dot(1);q_dot(2)]; 
        u = [0; K*(y_d-y)]; 
    end

    % forward dynamics and euler integration  
    q_ddot = inv(H)*(u - C);
    q_dot = q_dot + dt*q_ddot;
    q = q + dt*q_dot;

    qHistory(:,i) = q;
end


%% animation

figure
Ax = [0,0]; Ay = [0,0];
title('Double pendulum v1');
axis([-0.5 0.5 -0.5 0.5])
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

l1 = acrobot.l1;
l2 = acrobot.l2;

samplingTime = 10;

for i = 1 : samplingTime : n-1
    q1 = qHistory(1,i);
    q2 = qHistory(2,i);

    x1 = l1*sin(q1);
    y1 = -l1*cos(q1);

    x2 = x1+l2*sin(q1+q2);
    y2 = y1-l2*cos(q1+q2);

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