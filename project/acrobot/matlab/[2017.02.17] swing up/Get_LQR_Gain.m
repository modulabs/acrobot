clear;
clc;


% m1 = 10;
% m2 = 10;
% l1 = 10;
% l2 = 10;
% i1 = 1/12*m1*l1^2;
% i2 = 1/12*m2*l2^2;
% l_c1 = l1/2;
% l_c2 = l2/2;
% g = 9.8;

m1 = 1.9008;
m2 = 0.7175;
l1 = 0.2;
l2 = 0.2;
i1 = 4.3399*10^-3;
i2 = 5.2285*10^-3;
l_c1 = 1.8522*10^-1;
l_c2 = 6.2052*10^-2;
g = 9.8;

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
X = [q1 q2 d1_q1 d1_q2];

X_0 = [pi/2 0 0 0];
tau_0 = 0;

d2_q1 = d12/(d12^2-d11*d22)*(tau-h2-phi2+d12/d11*h1+d12/d11*phi1)-h1/d11-phi1/d11; % refer sacanned note
d2_q2 = 1/(d22-d12^2/d11)  *(tau-h2-phi2+d12/d11*h1+d12/d11*phi1);

d1_X = [d1_q1 d1_q2 d2_q1 d2_q2];

a = inline(jacobian(d1_X,X),'q1','q2','d1_q1','d1_q2','tau'); % jacobian = diffretiate vector by vector
b = inline(jacobian(d1_X,tau),'q1','q2','d1_q1','d1_q2','tau');

A = a(X_0(1),X_0(2),X_0(3),X_0(4),tau_0)
B = b(X_0(1),X_0(2),X_0(3),X_0(4),tau_0)

%%%%%%%%%%%%%%  lqr function  %%%%%%%%%%%%%%
Q =[ 1000 -500    0    0 
     -500 1000    0    0
        0    0 1000 -500
        0    0 -500 1000];
R = 10000;

K = lqr(A,B,Q,R)

