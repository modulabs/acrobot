clc; clear all; clf

% System Paramters
m = 0.3; b = 0.1; l = 0.5; g=9.81; u=1;

% Simplification
r = g/l;
k = b/(m*l);
p = 1/(m*l^2);

% The Pendulum ODE

% Initial Condition
x = [pi 0]+0.2*randn(1,2);


% Linearize the system
dt = 0.01;
kp = 100;
temp = zeros(100,2);
for i=1:100
    u = kp*(pi-x(1));
    f = [x(2); p*u - k*x(2) - r*sin(x(1))];
    x(i+1,:) = x(i,:) + dt*f';
%     x(i+1,:) = x(i,:);
end

O = [0 0];



% Loop for animation
for i=1:101
         
    
    % draw pendulum
    subplot(1,2,1)
    
    % title
    text(1, 1.3,'\bf LQR control for simple pendulum','HorizontalAlignment','center','VerticalAlignment', 'top')
    
    title('simple pendulum','fontsize',12)
    xlabel('x','fontsize',12)
    ylabel('y','fontsize',12)
    axis(gca, 'equal') % Aspect ratio of the plot
    axis([-0.7 0.7 -0.7 0.7 ]) % The limit of plot
 
    % Mass point
    P = l*[sin(x(i,1)) -cos(x(i,1))];
    
    % Circle in origin
    O_circ = viscircles(O, 0.01);
    
    % Pendulum
    pend = line([O(1) P(1)],[O(2) P(2)]);
    
    % Ball
    ball = viscircles(P, 0.05);
    
    % Time interval to update plot
    pause(0.001);
    
    % Delete previous object if it is not the final loop
    if i<101
        delete(pend);
        delete(ball);
        delete(O_circ);
    end
    
    % Phase Portrait
    
    % temp = [x(i,1) x(i,2)];
    subplot(1,2,2)
    axis(gca, 'equal')
    % axis([-10 10 -10 10])
    plot(x(:,1),x(:,2),'r')
    % plot(temp(:,1),temp(:,2),'r')
    hold on
    
    
    
    
    title('phase portrait','fontsize',12)
    xlabel('\theta','fontsize',12)
    ylabel('\thetadot','fontsize',12)
    


    
end