
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is for the simulation of a three arm robot 
% with revolute joints                                 
%
% This is a property of Feinberg Industries
% All rights reserved © 1873
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


close all; clear all; clc;
%% Create System Variables
syms t1;
syms t2;
syms t3;
syms t1_dot;
syms t2_dot;
syms t3_dot;
syms t;
a1 = 1;
a2 =1;
a3 = 1;
g = 9.81;
Tau = [0,0,0];


A_1 = [ cos(t1), -sin(t1), 0, a1 * cos(t1); ...
        sin(t1), cos(t1), 0, a1 * sin(t1); ...      
        0, 0, 1, 0; ...
        0, 0, 0, 1;];
A_2 = [ cos(t2), -sin(t2), 0, a2 * cos(t2); ...
        sin(t2), cos(t2), 0, a2 * sin(t2); ...      
        0, 0, 1, 0; ...
        0, 0, 0, 1;];
A_3 = [ cos(t3), -sin(t3), 0, a3 * cos(t3); ...
        sin(t3), cos(t3), 0, a3 * sin(t3); ...      
        0, 0, 1, 0; ...
        0, 0, 0, 1;];  
T_02 = simplify(A_1* A_2)
T_03 = simplify(T_02 * A_3)
z = [0; 0; 1];


x = [1;1;1;];
y = [1;1;1;];
z = [1;1;1;];
m = [1;1;1;];


% Calculate Inertia Matrix
I(1,:,:) =[m(1)*(y(1)^2+z(1)^2)/12, 0 , 0; ...
       0, m(1)*(x(1)^2+z(1)^2)/12, 0; ...
       0, 0, m(1)*(x(1)^2+y(1)^2)/12; ];
   
I(2,:,:) = [m(2)*(y(2)^2+z(2)^2)/12, 0 , 0; ...
   0, m(2)*(x(2)^2+z(2)^2)/12, 0; ...
   0, 0, m(2)*(x(2)^2+y(2)^2)/12; ];

I(3,:,:) =[m(3)*(y(3)^2+z(3)^2)/12, 0 , 0; ...
   0, m(3)*(x(3)^2+z(3)^2)/12, 0; ...
   0, 0, m(3)*(x(3)^2+y(3)^2)/12; ];


% Define Jacobian Matricies
Jw(1,:,:) = [0,0,0;0,0,0;1,0,0];
Jw(2,:,:) = [0,0,0;0,0,0;1,1,0];
Jw(3,:,:) = [0,0,0;0,0,0;1,1,1];   

x1 = a1*cos(t1)/2;
y1 = a1*sin(t1)/2;
z1 = 0;

Jv(1,:,:) = [diff(x1,t1),diff(x1,t2),diff(x1,t3);...
    diff(y1,t1),diff(y1,t2),diff(y1,t3);...
    diff(z1,t1),diff(z1,t2),diff(z1,t3)];

x2 = a1*cos(t1) +a2*cos(t1+t2)/2;
y2 = a1*sin(t1) +a2*sin(t1+t2)/2;
z2 = 0;

Jv(2,:,:) = [diff(x2,t1),diff(x2,t2),diff(x2,t3);...
    diff(y2,t1),diff(y2,t2),diff(y2,t3);...
    diff(z2,t1),diff(z2,t2),diff(z2,t3)];

x3 = a1*cos(t1) +a2*cos(t1+t2) +a3*cos(t1+t2+t3)/2;
y3 = a1*sin(t1) +a2*sin(t1+t2) +a3*sin(t1+t2+t3)/2;
z3 = 0;

Jv(3,:,:) = [diff(x3,t1),diff(x3,t2),diff(x3,t3);...
    diff(y3,t1),diff(y3,t2),diff(y3,t3);...
    diff(z3,t1),diff(z3,t2),diff(z3,t3)];

%Define U and T

U = m(1)*g*y1 + m(2)*g*y2 +m(2)*g*y3;

q_dot = [t1_dot;t2_dot;t3_dot];
q = [t1;t2;t3];

T = m(1).*q_dot'.*Jv(1,:,:)'.*Jv(1,:,:).*q_dot +...
    q_dot'.*Jw(1,:,:)'.*I(1,:,:).*Jw(1,:,:).*q_dot +...
    m(2).*q_dot'.*Jv(2,:,:)'.*Jv(2,:,:).*q_dot +...
    q_dot'.*Jw(2,:,:)'.*I(2,:,:).*Jw(2,:,:).*q_dot +...
    m(3).*q_dot'.*Jv(3,:,:)'.*Jv(3,:,:).*q_dot +...
    q_dot'.*Jw(3,:,:)'.*I(3,:,:).*Jw(3,:,:).*q_dot; 

%Lagrangian
L = T-U;

Tau = diff(diff(L,q_dot),t) - diff(L,q);




