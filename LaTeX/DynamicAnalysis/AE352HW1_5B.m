%% AE 352 HW 1
%  HW 1 Part B
close all; 
clear all;
% Convert KM to m/s
KM_TO_MPS = 1/3.6;
% Enter Speed plateaus
V0 = 810 * KM_TO_MPS;
V1 = 650 * KM_TO_MPS;
V2 = 390 * KM_TO_MPS;
% Find accelerations
dv0 = (V1-V0)/20;
dv1 = (V2-V1)/10;
% Angle Mark
a1 = pi/4;
% Angular Velocities
da0 = a1/20;
da1 = a1/10;
% Discretize Time
dt = 60/1000;
% Book keeping for velocity, gravity, time, and angles
v(1) = V0;
g(1) = 9.81;
t(1) = 0;
OMEGA = da0;
angle(1) = 0;

for i = 1:1000
    % increment time
    t(i+1) = t(i)+dt;
    % update velocity
    if(i*dt <= 20)
        v(i+1) = v(i) + dt*dv0;
    elseif(i*dt > 20 && i*dt <=30)
        v(i+1) = v(i) + dt*dv1;
    elseif(i*dt > 30 && i*dt <=40)
        v(i+1) = v(i) - dt*dv1;
    elseif(i*dt > 40 && i*dt <=60)
        v(i+1) = v(i) - dt*dv0;
    end
    % update angle
    if(i*dt <= 20)
        angle(i+1) = angle(i) + OMEGA*dt;
    elseif(i*dt > 20 && i*dt <=30)
        angle(i+1) = angle(i) - da1*dt;
    elseif(i*dt > 30 && i*dt <=40)
        angle(i+1) = angle(i) - da1*dt;
    elseif(i*dt > 40 && i*dt <=60)
        angle(i+1) = angle(i) + OMEGA*dt;
    end
    % update gravity
    if(i*dt <= 20)
        g(i+1) = (g(1) + v(i+1)*da0 *cos(angle(i+1)))/9.81;
    elseif(i*dt > 20 && i*dt <=30)
         g(i+1) = (g(1) - abs(v(i+1)*da1*cos(angle(i+1))))/9.81;  
    elseif(i*dt > 30 && i*dt <=40)
        g(i+1) = (g(1) - abs(v(i+1)*da1*cos(angle(i+1))))/9.81;
    elseif(i*dt > 40 && i*dt <=60)
        g(i+1) = (g(1) + v(i+1)*da0 *cos(angle(i+1)))/9.81;
    end
end
figure (1)
plot(t, g, 'bs');
title('Acceleration vs. Time');
ylabel('Acceleration (gs)');
xlabel('Time (s)');
ylim([-0.2, 2]);
xlim([0, 60]);