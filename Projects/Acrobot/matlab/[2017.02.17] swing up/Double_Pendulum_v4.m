%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverted pendulum - 2 links(rigid)
% Made by Gangnam Dynamics
% 2017. 02. 10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%%
% m1 = 10;    %mass of the link1(kg)
% m2 = 10;    %mass of the link2(kg)
% 
% l1 = 10;    %length of the link1(m)
% l2 = 10;    %length of the link2(m)
% 
% l_c1 = l1/2; %center of mass
% l_c2 = l2/2;
% 
% i1 = 1/12*m1*l1^2; %inertia
% i2 = 1/12*m2*l2^2;
% 
% g = 9.81;  %Gravity accelation(m/s^2)

m1 = 1.9008;
m2 = 0.7175;
l1 = 0.2;
l2 = 0.2;
i1 = 4.3399*10^-3;
i2 = 5.2285*10^-3;
l_c1 = 1.8522*10^-1;
l_c2 = 6.2052*10^-2;
g = 9.8;


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

torque = zeros(1,n);
controller = zeros(1,n);

X = zeros(4,n); %state vector
D1_X = zeros(4,n); % first diffrential state vector

%% Initial Condition
theta1(1) = 88*(pi/180);   %radian
D_theta1(1) = 0*(pi/180);

theta2(1) = 0*(pi/180);
D_theta2(1) = 0*(pi/180);

X(:,1) = [theta1(1) theta2(1) D_theta1(1) D_theta2(1)]';
D1_X(:,1) = [D_theta1(1) D_theta2(1) D2_theta1(1) D2_theta2(1)]';

alpha = 200;

des_d2q2 = 0;
des_d1q2 = 0;
% des_q2 = 0;   %radian

kp = 50;
kd = 50;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  LQR  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%  variables for function  %%%%%%%%%%%%%%
syms q1 q2 d1_q1 d1_q2 tau; 

%%%%%%%%%%%%%%  dynamics  %%%%%%%%%%%%%%
d11 = m1*l_c1^2 + m2 * (l1^2 + l_c2^2 + 2*l1*l_c2*cos(q2)) + i1 + i2;
d22 = m2*l_c2^2 + i2;
d12 = m2*(l_c2^2 + l1*l_c2*cos(q2))+i2;
h1 = -m2*l1*l_c2*sin(q2)*d1_q2^2-2*m2*l1*l_c2*sin(q2)*d1_q2*d1_q1;
h2 = m2*l1*l_c2*sin(q2)*d1_q1^2;
phi1=(m1*l_c1 + m2*l1)*g*cos(q1)+m2*l_c2*g*cos(q1+q2);
phi2=m2*l_c2*g*cos(q1+q2);

%%%%%%%%%%%%%%  state vecotr  %%%%%%%%%%%%%%
X_ = [q1 q2 d1_q1 d1_q2];

X_0 = [pi/2 0 0 0]';
tau_0 = 0;

d2_q1 = d12/(d12^2-d11*d22)*(tau-h2-phi2+d12/d11*h1+d12/d11*phi1)-h1/d11-phi1/d11; % refer sacanned note
d2_q2 = 1/(d22-d12^2/d11)  *(tau-h2-phi2+d12/d11*h1+d12/d11*phi1);

d1_X_= [d1_q1 d1_q2 d2_q1 d2_q2];

a = inline(jacobian(d1_X_,X_),'q1','q2','d1_q1','d1_q2','tau'); % jacobian = diffretiate vector by vector
b = inline(jacobian(d1_X_,tau),'q1','q2','d1_q1','d1_q2','tau');

A_ = a(X_0(1),X_0(2),X_0(3),X_0(4),tau_0);
B_ = b(X_0(1),X_0(2),X_0(3),X_0(4),tau_0);

%%%%%%%%%%%%%%  lqr function  %%%%%%%%%%%%%%
Q =[ 1000 -500    0    0 
     -500 1000    0    0
        0    0 1000 -500
        0    0 -500 1000];
R = 10000;

K = lqr(A_,B_,Q,R);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


for i=1 : 1 : n-1 

    M = [((1/3)*m1*l1^2+m2*l1^2+(1/3)*m2*l2^2+m2*l1*l2*cos(theta2(i)))   ((1/3)*m2*l2^2+0.5*m2*l1*l2*cos(theta2(i)));
                 ((1/3)*m2*l2^2+0.5*m2*l1*l2*cos(theta2(i)))                     ((1/3)*m2*l2^2)                   ];

    C = [            0                  (-0.5*m2*l1*l2*sin(theta2(i)));
         0.5*m2*l1*l2*sin(theta2(i))                 0               ];

    G = [(-m2*l1*l2*sin(theta2(i)))         0      ;
                     0                      0     ];

    A = [((0.5*m1)+m2)*g*l1*cos(theta1(i))+0.5*m2*g*l2*cos(theta1(i)+theta2(i));
                       0.5*m2*g*l2*cos(theta1(i)+theta2(i))                   ];

    M22_bar = M(2,2) - M(2,1)*inv(M(1,1))*M(1,2);
    h2_bar = 0 - M(2,1)*inv(M(1,1))*G(1);
    pi2_bar = A(2) - M(2,1)*inv(M(1,1))*A(1);

    des_q2 = 2*alpha/pi*atan(D_theta1(i)*pi/180);   %radian
    v2 = des_d2q2 + kd*(des_d1q2-D_theta2(i)) + kp*(des_q2-theta2(i));
    T1 = 0;

    if abs(wrapToPi(theta1(i))-pi/2) > 3*(pi/180)
      T2 = M22_bar*v2 + h2_bar + pi2_bar;
    else
      T2 = -K*(X(:,i)-X_0);
    end
    
    T = [T1;T2];
    torque(i) = T2;
    
    D2 = inv(M) * (T - C*[(D_theta1(i))^2;(D_theta2(i))^2]-G*[D_theta1(i)*D_theta2(i) ; D_theta2(i)*D_theta1(i)] - A);

    D2_theta1(i+1) = D2(1);
    D2_theta2(i+1) = D2(2);

    %Euler method
    D_theta1(i+1) = D_theta1(i) + dt*D2_theta1(i);
    theta1(i+1) = theta1(i) + dt*D_theta1(i);

    D_theta2(i+1) = D_theta2(i) + dt*D2_theta2(i);
    theta2(i+1) = theta2(i) + dt*D_theta2(i);

    X(:,i+1) = [theta1(i+1) theta2(i+1) D_theta1(i+1) D_theta2(i+1)];
    D1_X(:,i+1) = [D_theta1(i+1) D_theta2(i+1) D2_theta1(i+1) D2_theta2(i+1)];
    
    if  X(:,i+1)== X_0
        break;
    end
    
end


% figure(1);
% plot(t,theta1*180/pi,t,theta2*180/pi);
% legend('th1','th2');
% grid on

%% animation

figure(2)
Ax = [0,0]; Ay = [0,0];
title('Double pendulum v1');
% axis([-25 25 -25 25])
axis([-1 1 -1 1])
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







