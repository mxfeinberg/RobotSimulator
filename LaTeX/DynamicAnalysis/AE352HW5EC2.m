%%
% AE 352
% HW 5
% 2C EC
close all; clear all;

R = 10; %define initial Radius
v_0 = 10; %define initial speed
g = -9.81; %gravity constant

theta(1) = 0; % starting parameters
omega(1) = v_0/R;

dt = 100/1000;% time discretization
t_Max = 100;

x(1) = 0;
y(1) = R;
z(1) = 10;

vx(1) = 0;
vy(1) = v_0;
vz(1) = 0;
az(1) = 0;


for t=1 : 1000
 
 %utilize a ratio of radii (r_0)^2/r^2 to determine omega, 
 %then use omega^2*r to find centripetal acceleration
 az(t+1) = (omega(1)*(100/(R^2-(R-z(t))^2)))^2*(R-z(t));
 vz(t+1) = vz(t) + dt*((g)+az(t+1)); %alter gravity influence with centripetal
 z(t+1) = z(t) + dt*vz(t+1); %numerically integrate to find z
 
 omega(t+1) = omega(1)*(100/(R^2-(R-z(t+1))^2)); % find omega as before with radii ratio
 
 theta(t+1) = theta(t) +dt*omega(t+1); %numerically integrate to find theta
 
 x(t+1) = sin(theta(t+1))*sqrt(R^2-(R-z(t+1))^2); % numerically intrage to find new x and y
 y(t+1) = cos(theta(t+1))*sqrt(R^2-(R-z(t+1))^2);
 
end

%make circles for visualization purposes
x1(1)=0;
y1(1)=0;
z1(1)=0;
x2(1)=0;
y2(1)=0;
z2(1)=0;
for t = 1:360
    x1(t) = 10*cosd(t-1);
    x2(t) = 0;
    y1(t) = 0;
    y2(t) = 10*cosd(t-1);
    z1(t) = 10*sind(t-1)+10;
    z2(t) = 10*sind(t-1)+10;
end

figure(1)
title('Particle Trajectory in 3 Space');
hold on;
grid on;
plot3(x,y,z,'g-');
plot3(x1,y1,z1,'b-');
plot3(x2,y2,z2,'r-');