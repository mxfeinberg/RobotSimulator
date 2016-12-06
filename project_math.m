
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
syms a1;
syms a2;
syms a3;


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
O

x = [1,1,1];
y = [1,1,1];
z = [1,1,1];
m = [1,1,1];

I(1,:,:) =[m(1)*(y(1)^2+z(1)^2)/12, 0 , 0; ...
       0, m(1)*(x(1)^2+z(1)^2)/12, 0; ...
       0, 0, m(1)*(x(1)^2+y(1)^2)/12; ];
   
I(2,:,:) = [m(2)*(y(2)^2+z(2)^2)/12, 0 , 0; ...
   0, m(2)*(x(2)^2+z(2)^2)/12, 0; ...
   0, 0, m(2)*(x(2)^2+y(2)^2)/12; ];

I(3,:,:) =[m(3)*(y(3)^2+z(3)^2)/12, 0 , 0; ...
   0, m(3)*(x(3)^2+z(3)^2)/12, 0; ...
   0, 0, m(3)*(x(3)^2+y(3)^2)/12; ];
   
   









