
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

h1 = 1;
d1 = 1;
w1 = 1;
h2 = 1;
d2 = 1;
w2 = 1;
h3 = 1;
d3 = 1;
w3 = 1;

%% Define Matricies
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







