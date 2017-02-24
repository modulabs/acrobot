%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2 links pendulum(rigid)
% Swing up control - Non Collocated control
% Made by Gangnam Dynamics
% 2017. 02. 24
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

%matrix
D2_theta1 = zeros(1,n);
D_theta1 = zeros(1,n);
theta1 = zeros(1,n);

D2_theta2 = zeros(1,n);
D_theta2 = zeros(1,n);
theta2 = zeros(1,n);

%% Initial Condition
theta1(1) = -90*(pi/180);   %radian
D_theta1(1) = 0*(pi/180);

theta2(1) = 0*(pi/180);
D_theta2(1) = 0*(pi/180);

% alpha = 200;

des_d2q1 = 0;
des_d1q1 = 0;
des_q1 = 90*(pi/180);
% des_d2q2 = 0;
% des_d1q2 = 0;
% des_q2 = 0;   %radian

kp = 1;
kd = 1;

for i=1 : 1 : n-1 
M = [((1/3)*m1*l1^2+m2*l1^2+(1/3)*m2*l2^2+m2*l1*l2*cos(theta2(i)))   ((1/3)*m2*l2^2+0.5*m2*l1*l2*cos(theta2(i)));
             ((1/3)*m2*l2^2+0.5*m2*l1*l2*cos(theta2(i)))                     ((1/3)*m2*l2^2)                   ];
 
H = [(-0.5)*m2*l1*l2*sin(theta2(i))*D_theta1(i)*D_theta1(i) + (-m2)*l1*l2*(sin(theta2(i)))*D_theta1(i)*D_theta2(i);
                           0.5*m2*l1*l2*sin(theta2(i))*D_theta1(i)*D_theta1(i)                                   ];
                      
P = [((0.5*m1)+m2)*g*l1*cos(theta1(i))+0.5*m2*g*l2*cos(theta1(i)+theta2(i));
                   0.5*m2*g*l2*cos(theta1(i)+theta2(i))                   ];

Mt12 = M(1,2)'*inv(M(1,2)*M(1,2)'); % pseudo-inverse matrix, ref : http://www.matlabinuse.com/Mastering_MATLAB/11101

M21_bar = M(2,1) - M(2,2)*Mt12*M(1,1);
h2_bar = H(2) - M(2,2)*Mt12*H(1);
pi2_bar = P(2) - M(2,2)*Mt12*P(1);

v1 = des_d2q1 + kd*(des_d1q1-D_theta1(i)) + kp*(des_q1-theta1(i)); % control input
T1 = 0;
T2 = M21_bar*v1 + h2_bar + pi2_bar;

T = [T1;T2];

D2 = inv(M) * (T - H - P);

D2_theta1(i+1) = D2(1);
D2_theta2(i+1) = D2(2);

%Euler method
D_theta1(i+1) = D_theta1(i) + dt*D2_theta1(i+1);
theta1(i+1) = theta1(i) + dt*D_theta1(i+1);

D_theta2(i+1) = D_theta2(i) + dt*D2_theta2(i+1);
theta2(i+1) = theta2(i) + dt*D_theta2(i+1);
    
end

figure(1);
plot(theta1*180/pi,D_theta1*180/pi);
title('Phase portrait(q1,q1_dot)');
legend('q1');
grid on

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

x1 = l1*cos(theta1(i));
y1 = l1*sin(theta1(i));

x2 = x1+l2*cos(theta1(i)+theta2(i));
y2 = y1+l2*sin(theta1(i)+theta2(i));

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







