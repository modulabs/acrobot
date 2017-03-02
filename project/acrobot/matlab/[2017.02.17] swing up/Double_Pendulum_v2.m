%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverted pendulum - 2 links(rigid)
% Made by Gangnam Dynamics
% 2017. 02. 10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%matrix
ddq1 = zeros(1,n);
dq1 = zeros(1,n);
q1 = zeros(1,n);

ddq2 = zeros(1,n);
dq2 = zeros(1,n);
q2 = zeros(1,n);

%% Initial Condition
q1(1) = -45*(pi/180);   %radian
dq1(1) = 0*(pi/180);

q2(1) = 0*(pi/180);
dq2(1) = 0*(pi/180);

alpha = 200;

des_ddq2 = 0;
des_dq2 = 0;
% des_q2 = 0;   %radian

kp = 10;
kd = 10;

for i=1 : 1 : n-1 
    M = [((1/3)*m1*l1^2+m2*l1^2+(1/3)*m2*l2^2+m2*l1*l2*cos(q2(i)))   ((1/3)*m2*l2^2+0.5*m2*l1*l2*cos(q2(i)));
                 ((1/3)*m2*l2^2+0.5*m2*l1*l2*cos(q2(i)))                     ((1/3)*m2*l2^2)                   ];

    C = [            0                  (-0.5*m2*l1*l2*sin(q2(i)));
         0.5*m2*l1*l2*sin(q2(i))                 0               ];

    G = [(-m2*l1*l2*sin(q2(i)))         0      ;
                     0                      0     ];

    A = [((0.5*m1)+m2)*g*l1*cos(q1(i))+0.5*m2*g*l2*cos(q1(i)+q2(i));
                       0.5*m2*g*l2*cos(q1(i)+q2(i))                   ];


    M22_bar = M(2,2) - M(2,1)*inv(M(1,1))*M(1,2);
    h2_bar = 0 - M(2,1)*inv(M(1,1))*G(1);
    pi2_bar = A(2) - M(2,1)*inv(M(1,1))*A(1);

    des_q2 = 2*alpha/pi*atan(dq1(i)*pi/180);   %radian
    v2 = des_ddq2 + kd*(des_dq2-dq2(i)) + kp*(des_q2-q2(i));
    tau(1) = 0;
    tau(2) = M22_bar*v2 + h2_bar + pi2_bar;

    D2 = inv(M) * (tau - C*[(dq1(i))^2;(dq2(i))^2]-G*[dq1(i)*dq2(i) ; dq2(i)*dq1(i)] - A);

    ddq1(i+1) = D2(1);
    ddq2(i+1) = D2(2);

    %Euler method
    dq1(i+1) = dq1(i) + dt*ddq1(i+1);
    q1(i+1) = q1(i) + dt*dq1(i);

    dq2(i+1) = dq2(i) + dt*ddq2(i+1);
    q2(i+1) = q2(i) + dt*dq2(i);
    
end

% figure(1);
% plot(t,q1*180/pi,t,q2*180/pi);
% legend('th1','th2');
% grid on

%% animation

figure(2)
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

for i = 1 : 5 : n

x1 = l1*cos(q1(i));
y1 = l1*sin(q1(i));

x2 = x1+l2*cos(q1(i)+q2(i));
y2 = y1+l2*sin(q1(i)+q2(i));

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







