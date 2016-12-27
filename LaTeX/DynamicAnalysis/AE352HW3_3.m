%%
% AE 352 Problem 3
tMin = 1; %starting time
tMax = 200; %ending time
dt = (tMax-tMin)/1000; % time steps
v = zeros(3, 1000); %allocate space for velocity
x = zeros(1000); % allocate space for x values
y = zeros(1000); % allocate space for y values
for i=1:1000 %iterate to find vx and vy values
    v(1, i) = -2*sin(dt*i*0.5)-cos(dt*i*0.5)*(dt*i);
    v(2, i) =  1 - sin(dt*i*0.5)*(i*dt)+2*cos(dt*i*0.5);
    v(3, i) = sqrt(v(1,i)^2 + v(2,i)^2); %find the magnitude of velocity from the components
end

for i = 1:999 %iterate to find positions
    x(i+1) = x(i) + (dt)*(v(1,i));
    y(i+1) = y(i) + dt*(v(2,i));
end
figure(1)
plot(x,y, 'b-');
title('Y vs X');
xlabel('x distance (m)');
ylabel('y distance (m)');

vecV = sqrt(x.^2 + y.^2);
figure(2)
t = 1:(199/999):200;
plot(t, v(3,:), 'r-')
title('V vs T');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
syms z
% Repeat the Process for a different Omega
v2 = zeros(3, 1000);
x2 = zeros(1000);
y2 = zeros(1000);

for i=1:1000
    theta = integral(@(z)0.5*exp(-0.01*z),1, i*dt); %% theta is no longer constant so we must integrate to find it
    %v2(1, i) = -2*sin(dt*i*(0.5*exp(-0.01*i*dt)))-cos(dt*i*0.5*exp(-0.01*i*dt))*(dt*i*2*0.5*exp(-0.01*i*dt));
    %v2(2, i) = 1 - sin(dt*i*0.5*exp(-0.01*i*dt))*(2*i*dt*0.5*exp(-0.01*i*dt))+2*cos(dt*i*0.5*exp(-0.01*i*dt));
    %v2(3, i) = sqrt(v2(1,i)^2 + v2(2,i)^2);
    v2(1, i) = -2*sin(theta)-cos(theta)*(dt*i*2*0.5*exp(-0.01*i*dt));
    v2(2, i) = 1 - sin(theta)*(2*i*dt*0.5*exp(-0.01*i*dt))+2*cos(theta);
    v2(3, i) = sqrt(v2(1,i)^2 + v2(2,i)^2);
end

for i = 1:999
    x2(i+1) = x2(i) + (dt)*(v2(1,i));
    y2(i+1) = y2(i) + dt*(v2(2,i));
end

figure(3)
plot(x2, y2, 'g-')
title('Y vs X');
xlabel('x distance (m)');
ylabel('y distance (m)');
figure(4)
plot(t, v2(3,:),'r-')
title('V vs T');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
%%instead of 2t, we'll try tanh(t) for the spiders movement
v3 = zeros(3, 1000);
x3 = zeros(1000);
y3 = zeros(1000);

for i=1:1000
    v3(1, i) = -2*sin(dt*i*0.5)-cos(dt*i*0.5)*(0.5*tanh(dt*i));
    v3(2, i) =  1 - sin(dt*i*0.5)*(0.5*tanh(dt*i))+2*cos(dt*i*0.5);
    v3(3, i) = sqrt(v3(1,i)^2 + v3(2,i)^2);
end

for i = 1:999
    x3(i+1) = x3(i) + (dt)*(v3(1,i));
    y3(i+1) = y3(i) + dt*(v3(2,i));
end

figure(5)
plot(x3, y3, 'g-')
title('Y vs X');
xlabel('x distance (m)');
ylabel('y distance (m)');
figure(6)
plot(t, v3(3,:),'r-')
title('V vs T');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
