%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2 links pendulum(rigid)
% Swing up control - Collocated control
% Made by Gangnam Dynamics
% 2017. 02. 20
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
clc;
close all;

%%
m1 = 10;    %mass of the link1(kg)
m2 = 10;    %mass of the link2(kg)

l1 = 10;    %length of the link1(m)
l2 = 10;    %length of the link2(m)

g = 9.81;  %Gravity accelation(m/s^2)
%%
dt = 0.001;    %time step
t = 0:dt:30;   %time
n = length(t);

%% Initial Condition
q = deg2rad([-45; 0]);%radian
dq = deg2rad([0; 0]);
ddq = zeros(2,1);

q_traj = q;

alpha = 200;

des_ddq2 = 0;
des_dq2 = 0;
% des_q2 = 0;   %radian

kp = 10;
kd = 10;

for i=1 : 1 : n-1 
    M = [((1/3)*m1*l1^2+m2*l1^2+(1/3)*m2*l2^2+m2*l1*l2*cos(q(2)))   ((1/3)*m2*l2^2+0.5*m2*l1*l2*cos(q(2)));
                 ((1/3)*m2*l2^2+0.5*m2*l1*l2*cos(q(2)))                     ((1/3)*m2*l2^2)                   ];

    H = [(-0.5)*m2*l1*l2*sin(q(2))*dq(1)*dq(1) + (-m2)*l1*l2*(sin(q(2)))*dq(1)*dq(2);
                               0.5*m2*l1*l2*sin(q(2))*dq(1)*dq(1)                                   ];

    P = [((0.5*m1)+m2)*g*l1*cos(q(1))+0.5*m2*g*l2*cos(q(1)+q(2));
                       0.5*m2*g*l2*cos(q(1)+q(2))                   ];


    M22_bar = M(2,2) - M(2,1)*inv(M(1,1))*M(1,2);
    h2_bar = H(2) - M(2,1)*inv(M(1,1))*H(1);
    pi2_bar = P(2) - M(2,1)*inv(M(1,1))*P(1);

    des_q2 = 2*alpha/pi*atan(dq(1)*pi/180);   %radian
    v2 = des_ddq2 + kd*(des_dq2-dq(2)) + kp*(des_q2-q(2));
    
    tau = [0; M22_bar*v2 + h2_bar + pi2_bar];

    ddq = inv(M) * (tau - H - P);

    % Euler method
    dq = dq + dt*ddq;
    q = q + dt*dq;
    
    q_traj = [q_traj q];
    
end

% figure(1);
% plot(t,q1*180/pi,t,q2*180/pi);
% legend('th1','th2');
% grid on

%% animation

figure(2)
Ax = [0,0]; Ay = [0,0];
title('Double pendulum v1');
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

for i = 1 : 5 : n

    x1 = l1*cos(q_traj(1,i));
    y1 = l1*sin(q_traj(1,i));

    x2 = x1+l2*cos(q_traj(1,i)+q_traj(2,i));
    y2 = y1+l2*sin(q_traj(1,i)+q_traj(2,i));

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
