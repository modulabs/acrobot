%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamic Simulation
% Baek-Kyu Cho & DongHyun Ahn, Robotics & Control Lab,Kookmin University
% 2014.05.28.wed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;

global p1 p2;

%% variable setting 
m = 1;         % Mass of Bottom Link(kg) 
l = 1;         % Length of Link(m)
g = 9.81;      % Gravitational acceleration(m/s^2)
b = 1; 
%%
dt = 0.05;
t = 0:dt:30;
n = length(t);

theta = zeros(1,n);
D_theta = zeros(1,n);
D2_theta = zeros(1,n);

%initial condition
theta(1) = 15*rand*pi/180; %radian, 5 degrees
D_theta(1) = 0;
D2_theta(1) = 0;
Kp = 100;


A = [0,1;g/l,-b/(m*l^2)];
B = [0;1/m*l^2];
Q = [10 0;0 1];
R = 5;
k = lqr(A,B,Q,R);

for i = 1 : n-1
    time = n*dt;
    u = -Kp*(theta(i)-45*pi/180);
    % u = -k*[theta(i); D_theta(i)];
    D2_theta(i+1) = (g/l)*sin(theta(i)) + (1/(m*l^2))*u -(b/m*l^2)*D_theta(i);
    D_theta(i+1) = D_theta(i) + D2_theta(i+1)*dt;
    theta(i+1) = theta(i) + D_theta(i+1)*dt;
    
end

% figure(1);
% plot(t,theta*180/pi);
% grid on

% Phase portait
figure(3);
plot(theta*180/pi, D_theta*180/pi);
grid on

%% Animation

figure(2);
Ax=[0,0]; Ay=[0,0]; Az=[0,0];
xlabel('length(m)')
ylabel('length(m)')
axis equal

p1 = line(Ax,Ay,'Linewidth',3,'color','r'); %pole
hold on
p2 = plot(Ax,Ay,'o','MarkerSize',10,'MarkerFaceColor','r'); %mass

xlim([-1.5,1.5]);
ylim([-1.5,1.5]);
grid on

view([0,0,1])

for i=1:2:n-1
    a_draw_animation(theta(i));
end

print('end');

