%%
% AE 352 
% HW 2 Question 3
% Max Feinberg
function main
R0 = eye(3,3); % initialize identity matrix 
tMax = 6; % send end time
tRate = 30; % define time discretizations
[t,x] = ode45(@f, linspace(0, tMax, tRate*tMax), RtoX(R0)); %numerically integrate
figure(1) %plot
plot(t, x);
title('Rotation Matrix Values vs. Time');
ylabel('Rotation Matrix Values');
xlabel('Time (s)');
disp(x)
R1 = [0;0;0;]; %angle initial conditions
[time, theta] = ode45(@g, linspace(0, tMax, tRate*tMax), R1); % numerically integrate
figure(2)
plot(time, theta);
title('Angles vs. Time');
ylabel('Angles (rad)');
xlabel('Time (s)');
legend('\theta_{1}', '\theta_{2}', '\theta_{3}'); 

% helper for part a
function xdot = f(t, x)
R = XtoR(x);
w = 10 * exp(-1*t)*[sin(t); sin(2*t); sin(3*t)];
Rdot = R * skew(w);
xdot = RtoX(Rdot);

% helper for part b
function thetaDot = g(t, theta)
w = 10 * exp(-1*t)*[sin(t); sin(2*t); sin(3*t)];
thetaDot = b(theta) * w;

% skews a given matrix by rearranging it
function S = skew(w)
S = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

% reshapes an input matrix to a vector
function X = RtoX(R)
X = reshape(R, 9, 1)

%reshapes an input vector to a matrix
function R = XtoR(X)
R = reshape(X, 3, 3)


% defines b, our pseudo 'rotation' matrix for part b
function beta = b(theta)
beta = (1/cos(theta(2)))*[cos(theta(3)), -1*sin(theta(3)), 0; ...
        cos(theta(2))*sin(theta(3)), cos(theta(2))*cos(theta(3)), 0; ...
        -1*sin(theta(2))*cos(theta(3)), sin(theta(2))*sin(theta(3)), cos(theta(2))];
