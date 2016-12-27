%%  AE 352 HW 7 
%   Problem 1-2
%   Author: Max Feinberg
%   Simulates the motion of a cylinder rolling in a hemisphere
%   and a stick falling
function main
clear all; close all; clear figure; clc; %stand clears

R = 1; % Ramp Radius
r = 0.2; % cylinder radius
g = 9.8; % gravity

t0 = 0;         % initial time
tf = 25;        % final time
phi_0 = -1*pi/2*R/r;
y0 = [-1*pi/2; 0; phi_0; 0];   % initial conditions

[t,y] = ode45(@diff1,[t0 tf],y0); % Use ode45 to solve

figure(1) % Plot theta vs t
plot(t,y(:,1));
xlabel('time (s)');
ylabel('\theta (radians)');
title('\theta vs. time');
grid on
print -depsc P1Theta

theta0 = [pi/3; 0;];
[u,v] = ode45(@diff2,[0 25],theta0); % simulate stick falling

figure(2)
plot(u,v(:,1));
xlabel('time (s)');
ylabel('\theta (radians)');
title('\theta vs. time');
grid on
print -depsc P2ThetaShort

end

function state = diff1(t,y)
R = 1;
r = 0.2;
g = 9.8;

state = [y(2); -1*2*g/(3*R)*sin(y(1)); y(4); -2*g/(3*r)*sin(y(1))]; 
end

function state = diff2(u,v)
L = 2;
g = 9.8;

state = [v(2); (6*(g/L)*sin(v(1))-3*v(2)^2*sin(v(1))*cos(v(1))) / (1 + 3 * (sin(v(1))^2))]; 
end
