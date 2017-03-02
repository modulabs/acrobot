function run_Euler()
clear all;
clc;
close all;

FinalTime = 30;         % sec
dt = 0.001;             % time step (sec)
t = 0:dt:FinalTime;     % time
n = length(t);

i1 = 4.3399*10^-3;
i2 = 5.2285*10^-3;
l_c1 = 1.8522*10^-1;
l_c2 = 6.2052*10^-2;


obj = AcrobotPlant;
obj.l1 = 0.2; 
obj.l2 = 0.2;  
obj.m1 = 1.9008; 
obj.m2 = 0.7175;  
obj.lc1 = l_c1; 
obj.lc2 = l_c2;
obj.Ic1 = i1;
obj.Ic2 = i2;

q = [pi+deg2rad(2);-deg2rad(2)]; % q = [pi+0.04;-0.05];
q_dot = [-0.5;0]; % q_dot = [-0.2;0.04];
q_dotdot = [0;0];

y = zeros(4,1);
y_d = [pi;0;0;0];

K = [-113.8908, -9.5070, -17.3951, -1.9405];

for i=1 : 1 : n-1 

y = [q(1);q(2);q_dot(1);q_dot(2)]; 
u = [0; K*(y_d-y)]; % u = [0;0];

[H,C,B] = manipulatorDynamics(obj,q,q_dot);
q_dotdot = inv(H)*(u - C)

q_dot = q_dot + dt*q_dotdot;
q = q + dt*q_dot;

qData(:,i) = q;

end


%% animation

figure(1)


figure(2)
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

l1 = obj.l1;
l2 = obj.l2;

samplingTime = 10;

for i = 1 : samplingTime : n-1

    theta1 = qData(1,i);
    theta2 = qData(2,i);

    x1 = l1*sin(theta1); % x1 = l1*cos(theta1);
    y1 = -l1*cos(theta1); % y1 = l1*sin(theta1);

    x2 = x1+l2*sin(theta1+theta2); % x2 = x1+l2*cos(theta1+theta2);
    y2 = y1-l2*cos(theta1+theta2); % y2 = y1+l2*sin(theta1+theta2);

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

end