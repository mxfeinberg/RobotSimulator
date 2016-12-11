function Robot_simulation_352



params.action_filename = 'action.mat';
params.movie_filename = 'movie_3.avi';
params.snapshot_filename = 'snapshot.pdf';
params.makemovie = false;
params.makesnapshot = false;

% - NOTE: The keyboard interface is as follows:
%
%   'q'         causes the simulation to quit (gracefully, saving the
%               actions, the movie, and/or the snapshot)
% 
%   '1'         increases the torque applied to link 1 at the first joint
%   '!'         decreases the torque applied to link 1 at the first joint
%
%   '2'         increases the torque applied to link 2 at the second joint
%   '@'         decreases the torque applied to link 2 at the second joint
%
%   '3'         increases the torque applied to link 3 at the third joint
%   '#'         decreases the torque applied to link 3 at the third joint
%

% Define the geometry and mass properties of the robot.
robot = GetGeometryOfRobot;

% Run the simulation.
RunSimulation(robot,params,0,0,pi/3,0,0,0);
function [wedge] = wedge(vector)
wedge = [0 -vector(3) vector(2);
         vector(3) 0 vector(1);
         -vector(2) vector(1) 0];
function robot = GetGeometryOfRobot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Sets up the Geometry of the robot
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.base.dx=sqrt(2);
robot.base.dy=sqrt(2);
robot.base.dz=.5;
robot.base.p_in0 = [0.5*robot.base.dx*[-1 1 1 -1 -1 1 1 -1];
                    0.5*robot.base.dy*[-1 -1 -1 -1 1 1 1 1];
                    0.5*robot.base.dz*[-1 -1 1 1 -1 -1 1 1]];

robot.link1.dx=1;
robot.link1.dy=4;
robot.link1.dz=1;
robot.link1.p_in1 = [0.5*robot.link1.dx*[-1 1 1 -1 -1 1 1 -1];
                     0.5*robot.link1.dy*[-1 -1 -1 -1 1 1 1 1];
                     0.5*robot.link1.dz*[-1 -1 1 1 -1 -1 1 1]];

robot.link2.dx=1;
robot.link2.dy=4;
robot.link2.dz=1;
robot.link2.p_in2 = [0.5*robot.link2.dx*[-1 1 1 -1 -1 1 1 -1];
                     0.5*robot.link2.dy*[-1 -1 -1 -1 1 1 1 1];
                     0.5*robot.link2.dz*[-1 -1 1 1 -1 -1 1 1]];

robot.link3.dx=1;
robot.link3.dy=4;
robot.link3.dz=1;
robot.link3.p_in3 = [0.5*robot.link3.dx*[-1 1 1 -1 -1 1 1 -1];
                     0.5*robot.link3.dy*[-1 -1 -1 -1 1 1 1 1];
                     0.5*robot.link3.dz*[-1 -1 1 1 -1 -1 1 1]];

robot.link1.p_in0 = nan(size(robot.link1.p_in1));
robot.link2.p_in0 = nan(size(robot.link2.p_in2));
robot.link3.p_in0 = nan(size(robot.link3.p_in3));

robot.faces =  [1 2 3;
                3 4 1;
                2 6 7;
                7 3 2;
                6 5 8;
                8 7 6;
                5 1 4;
                4 8 5;
                4 3 7;
                7 8 4;
                5 6 2;
                2 1 5];


robot.link1.rho = 1;
robot.link2.rho = 1;
robot.link3.rho = 1;

alpha = 0.0;
beta = .5;
delta = 0.00;

robot.a1 = [(alpha+delta)*(robot.link1.dx+robot.base.dx);0;0];
robot.b1 = [0; beta*robot.link1.dy; 0];
robot.a2 = [(alpha+delta)*(robot.link2.dx+robot.link1.dx); beta*robot.link1.dy; 0];
robot.b2 = [0; beta*robot.link2.dy; 0];
robot.a3 = [(alpha+delta)*(robot.link3.dx+robot.link2.dx); beta*robot.link2.dy; 0];
robot.b3 = [0; beta*robot.link3.dy; 0];

robot.p_01in0 = [(alpha+delta)*robot.base.dz;0;0];
robot.p_12in1 = [(alpha+delta)*robot.link1.dx;beta*robot.link1.dy;0];
robot.p_23in2 = [(alpha+delta)*robot.link2.dx;beta*robot.link2.dy;0];

%The coefficient of friction at each joint
robot.kfriction = 1;

%Mass and moment of inertia of link #1
robot.link1.m = robot.link1.rho* robot.link1.dx * robot.link1.dy * robot.link1.dz;
robot.link1.J_in1 = robot.link1.m* [(robot.link1.dy^2 + robot.link1.dz^2)/12 0 0;
                                    0 (robot.link1.dx^2+robot.link1.dz^2)/12 0;
                                    0 0 (robot.link1.dy^2 + robot.link1.dx^2)/12];

%Mass and moment of inertia of link #2
robot.link2.m = robot.link2.rho* robot.link2.dx * robot.link2.dy * robot.link2.dz;
robot.link2.J_in2 =  robot.link2.m* [(robot.link2.dy^2 + robot.link2.dz^2)/12 0 0;
                                    0 (robot.link2.dx^2+robot.link2.dz^2)/12 0;
                                    0 0 (robot.link2.dy^2 + robot.link2.dx^2)/12];

%Mass and moment of inertia of link #3
robot.link3.m = robot.link3.rho* robot.link3.dx * robot.link3.dy * robot.link3.dz;
robot.link3.J_in3 =  robot.link3.m* [(robot.link3.dy^2 + robot.link3.dz^2)/12 0 0;
                                    0 (robot.link3.dx^2+robot.link3.dz^2)/12 0;
                                    0 0 (robot.link3.dy^2 + robot.link3.dx^2)/12];


function RunSimulation(robot,params,err,w2,ref,err2,c2,ref2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%Set up the robot and set initial conditions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
world = CreateFigure(robot,params);
% Start time
t = 0;
% time step.
dt = 2e-2;

% time limit.
tmax = 100;

% joint positions
theta = [0;0;0];
% joint velocities
thetadot = [0;0;0];
% intial torques
u1 = 0;
u2 = 0;
u3 = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Records actions and makes a movie
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global action done
action = [u1;u2;u3];
if (params.makemovie)
    load(params.action_filename);
    myV = VideoWriter(params.movie_filename);
    myV.Quality = 100;
    open(myV);
else
    actionRecord = [];
end

done = false;
while (~done)
    
    if (params.makemovie)
        [actionRecord,curaction,done] = RetrieveAction(actionRecord);
        action = curaction;
    else
        curaction = action;
        actionRecord = StoreAction(actionRecord,curaction);
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Computes the Input Torque
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    u1 = curaction(1);
   %%%%% PID for joint 1
%     k1 = 500;
%     k2 = 10;
%     k3  = 400;
%     
%     rate = 0;    
%     ref = ref + rate*dt;
%     w1 = ref-theta(1);
%     err = (ref - theta(1))+err;
%     diff =(w1-w2)/dt;
%     w2=w1;
%     u1 = k1*(ref-theta(1))+ k2*err+k3*diff;
    
    u2 = curaction(2);
    %%%% PID for joint 2
    rate2 = 0;
    ref2 = ref2+rate2*dt;
    ref2-theta(2);
    k21 = 500;
    k22 = 10;
    k23 = 200;
    c1 = ref2-theta(2);
    err2 = (ref - theta(2))+err2;
    diff2 = (c1-c2)/dt;
    c2 = c1;
    u2 = k21*(ref2-theta(2))+k22*err2;+k23*diff2;
    
    u3 = curaction(3);

    [t,theta,thetadot] = Simulate(t,dt,theta,thetadot,u1,u2,u3,robot);            
	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Calculates the position of the links in the initial frame
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    c1 = cos(theta(1));
    s1 = sin(theta(1));
 
    R_1in0 = [1 0 0;
              0 c1 -s1;
              0 s1 c1];
    o_1in0 = robot.a1 + R_1in0*robot.b1;
    
    s2 = sin(theta(2));
    c2 = cos(theta(2));
    R_2in1 = [1 0 0;
              0 c2 -s2;
              0 s2 c2];
    R_2in0 = R_1in0*R_2in1;
    o_2in1 = robot.a2 + R_2in1*robot.b2;
    o_2in0 = o_1in0 + R_1in0*o_2in1;
    
    s3 = sin(theta(3));
    c3 = cos(theta(3));
    R_3in2 = [1 0 0;
              0 c3 -s3
              0 s3 c3];
    R_3in0 = R_2in0*R_3in2;
    o_3in2 = robot.a3 + R_3in2*robot.b3;
    o_3in0 = o_2in0 + R_2in0*o_3in2;
    
    robot.link1.p_in0 = [o_1in0 o_1in0 o_1in0 o_1in0 o_1in0 o_1in0 o_1in0 o_1in0] + R_1in0*robot.link1.p_in1;
    robot.link2.p_in0 = [o_2in0 o_2in0 o_2in0 o_2in0 o_2in0 o_2in0 o_2in0 o_2in0] + R_2in0*robot.link2.p_in2;
    robot.link3.p_in0 = [o_3in0 o_3in0 o_3in0 o_3in0 o_3in0 o_3in0 o_3in0 o_3in0] + R_3in0*robot.link3.p_in3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Updates the world and stores figure for movie 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update the figure.
    world = UpdateFigure(world,robot,o_1in0,R_1in0,o_2in0,R_2in0,o_3in0,R_3in0,u1,u2,u3,t,tmax);
    
    % If making a movie, store the current figure as a frame.
    if (params.makemovie)
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    
    % Stop if time has reached its maximum.
    if (t>tmax)
        done = true;
    end
    
end
% Either close the movie or save the record of actions.
if (params.makemovie)
    for i=1:30
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
else
    save(params.action_filename,'actionRecord');
end

if (params.makesnapshot)
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',params.snapshot_filename);
end

function [thetadot,thetadotdot] = GetRates(theta,thetadot,u1,u2,u3,robot)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Calculates the theta double dots using ode45 and derived equations of
% motion
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% inputs:
%
%   theta       3x1 matrix of joint angles
%   thetadot    3x1 matrix of joint velocities
%   u1          torque applied by motor to link 1 through joint 1
%   u2          torque applied by motor to link 2 through joint 2
%   u3          torque applied by motor to link 3 through joint 3
%   robot       a whole bunch of parameters (see GetGeometryOfRobot)
%
% outputs:
%
%   thetadot    3x1 matrix of joint velocities
%   thetadotdot 3x1 matrix of joint accelerations
   
    g = 9.81;
    m1 = robot.link1.m;
    m2 = robot.link2.m;
    m3 = robot.link3.m;
    
    J1 = robot.link1.J_in1;
    J2 = robot.link2.J_in2;
    J3 = robot.link3.J_in3;
thetadot = thetadot;
%thetadotdot = gamma(1:3);
a1 = robot.link1.dy;
a2 = robot.link2.dy;
a3 = robot.link3.dy;
a1c = robot.link1.dy/2;
a2c = robot.link2.dy/2;
a3c = robot.link3.dy/2;
%beta = [.25*m1*o1^2+J1(3,3)+m2*(o1^2+.5*o2^2+o1*o2*cos(theta(2))+J2(3,3)),(m2*(.25*o2^2+.5*o1*o2*cos(theta(2)))+J2(3,3)) ; (m2*(.25*o2^2+.5*o1*o2*cos(theta(2))) +J2(3,3)), (.25*m2*o2^2+J2(3,3))];
%delta =[u1;u2] - [-m2*sin(theta(2)),-.5*m2*o1*o2*sin(theta(1));-.5*m2*o1*o2*sin(theta(1)),0]*[thetadot(1)*thetadot(2);thetadot(2)^2]...
   % -[.5*m1*o1*g*cos(theta(1))+m2*g*(.5*o2*cos(theta(1)+theta(2))+o1*cos(theta(1))) ; (.5*m2*o2*g*cos(theta(1)+theta(2)))];

%Gamma = beta\delta;
%thetadotdot = Gamma(1:2);


% G1 = .25*(-2*g*(o1*(m1*2*(m2+m3))*cos(theta(1))+o2*(m2+2*m3)*cos(theta(1)+theta(2)) +o3*m3*cos(theta(1)+theta(2)+theta(3))) -o1*(o2*(m2+2*m3)*sin(theta(2)) +o3*m3*sin(theta(2)+theta(3)))*thetadot(2)*...
%    (2*thetadot(1)+thetadot(2)) -2*o3*m3*(o2*sin(theta(3))+o1*sin(theta(2)+theta(3)))*(thetadot(1)+thetadot(2))*thetadot(3) - o3*m3*(o2*sin(theta(3)) +o1*sin(theta(2)+theta(3)))*thetadot(3)^2);
% 
% A3=(3/8)*(4*J3(3,3)+o3^2*m3+2*o3*m3*(o2*cos(theta(3))+o1*cos(theta(2)+theta(3))));
% A2=(3/8)*(4*J2(3,3)+4*J3(3,3)+o3^2*m3+o2^2*(m2+4*m3)+2*o1*o2*(m2+2*m3)*cos(theta(2))+...
%    2*o3*m3*(2*o2*cos(theta(3))+o1*cos(theta(2)+theta(3))));
% A1=(3/8)*(4*J1(3,3)+4*J2(3,3)+4*J3(3,3)+o3^2*m3+o2^2*(m2+4*m3)+o1^2*(m1+4*(m2+m3))+...
%    4*(o1*o2*(m2+2*m3)*cos(theta(2))+o3*m3*(o2*cos(theta(3))+o1*cos(theta(2)+theta(3)))));
% 
% 
% G3=(1/4)*o3*m3*(2*-g*cos(theta(1)+theta(2)+theta(3))-o2*sin(theta(3))*thetadot(2)*thetadot(3)-...
%     thetadot(1)*(o2*sin(theta(3))*thetadot(3)+o1*sin(theta(2)+theta(3))*(thetadot(2)+thetadot(3))));
% C1=(3/8)*(4*J3(3,3)+o3^2*m3+2*o3*m3*(o2*cos(theta(3))+o1*cos(theta(2)+theta(3))));
% C2=(3/8)*(4*J3(3,3)+o3^2*m3+2*o2*o3*m3*cos(theta(3)));
% C3=(3/8)*(4*J3(3,3)+o3^2*m3);
% 
% G2=(1/4)*(2*o2*-g*(m2+2*m3)*cos(theta(1)+theta(2))+2*o3*-g*m3*cos(theta(1)+theta(2)+theta(3))-...
%    o2*o3*m3*sin(theta(3))*thetadot(3)*(2*thetadot(2)+thetadot(3))+...
%    thetadot(1)*(-o1*(o2*(m2+2*m3)*sin(theta(2))+o3*m3*sin(theta(2)+theta(3)))*thetadot(2) -o3*m3*(2*o2*sin(theta(3))+o1*sin(theta(2)+theta(3)))*thetadot(3)));
% 
% B1=(3/8)*(4*J2(3,3)+4*J3(3,3)+o3^2*m3+o2^2*(m2+4*m3)+2*o1*o2*(m2+2*m3)*cos(theta(2))+2*o3*m3*(2*o2*cos(theta(3))+o1*cos(theta(2)+theta(3))));
% B2=(3/8)*(4*J2(3,3)+4*J3(3,3)+o3^2*m3+o2^2*(m2+4*m3)+4*o2*o3*m3*cos(theta(3)));
% B3=(3/8)*(4*J3(3,3)+o3^2*m3+2*o2*o3*m3*cos(theta(3)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Working 2 link case
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G1 = g*(a1c*m1+a1*m2)*cos(theta(1))+a2c*g*m2*cos(theta(1)+theta(2)) - a1*a2c*m2*sin(theta(2))*thetadot(2)*(2*thetadot(1)+thetadot(2));
A1 = (J1(3,3)+J2(3,3)+a1c^2*m1+(a1^2+a2c^2)*m2+2*a1*a2c*m2*cos(theta(2)));
A2 = (J2(3,3)+a2c^2*m2+a1*a2c*m2*cos(theta(2)));
A3 = 0;

G2 = a2c*m2*(g*cos(theta(1)+theta(2))+a1*sin(theta(2))*thetadot(1)^2);
B1 = (J2(3,3)+a2c^2*m2+a1*a2c*m2*cos(theta(2)));
B2 = (J2(3,3)+a2c^2*m2);
B3 = 0;

G3=0;
C1=0;
C2=0;
C3=0;


beta = [A1 A2 A3;B1 B2 B3;];
tau = [u1 - G1;u2 - G2;];
Gamma = beta\tau;

thetadotdot = Gamma(1:2);
thetadotdot(3) = 0;
%thetadotdot(2) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%Working 3 link case
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% G1 = a1c*g*m1*cos(theta(1))+a1*g*m2*cos(theta(1))+a1*g*m3*cos(theta(1))+a2c*g*m2*cos(theta(1)+theta(2))+a2*g*m3*cos(theta(1)+theta(2))+...
%     a3c*g*m3*cos(theta(1)+theta(2)+theta(3)) -a1*((a2c*m2+a2*m3)*sin(theta(2))+a3c*m3*sin(theta(2)+theta(3)))*thetadot(2)^2 -...
%     2*a3c*m3*(a2*sin(theta(3))+a1*sin(theta(2)+theta(3)))*thetadot(2)*thetadot(3) - a2*a3c*m3*sin(theta(3))*thetadot(3)^2 -a1*a3c*m3*sin(theta(2)+theta(3))*thetadot(3)^2-...
%     2*thetadot(1)*(a1*((a2c*m2+a2*m3)*sin(theta(2)) +a3c*m3*sin(theta(2)+theta(3)))*thetadot(2)+a3c*m3*(a2*sin(theta(3))+a1*sin(theta(2)+theta(3)))*thetadot(3));
% A3 = J3(3,3)+a3c^2*m3+a2*a3c*m3*cos(theta(3))+a1*a3c*m3*cos(theta(2)+theta(3));
% A2 = J2(3,3)+J3(3,3)+a2c^2*m2+a2^2*m3+a3c^2*m3+a1*(a2c*m2+a2*m3)*cos(theta(2))+...
%     2*a2*a3c*m3*cos(theta(3))+a1*a3c*m3*cos(theta(2)+theta(3));
% A1 = J1(3,3)+J2(3,3)+J3(3,3)+a1c^2*m1+a1^2*m2+a2c^2*m2+a1^2*m3+a2^2*m3+a3c^2*m3+2*a1*(a2c*m2+a2*m3)*cos(theta(2))+...
%     2*a2*a3c*m3*cos(theta(3))+2*a1*a3c*m3*cos(theta(2)+theta(3));
% 
% 
% G2 = a2c * g* m2*cos(theta(1) + theta(2))...
%     + a2 * g * m3*cos(theta(1)+theta(2))...
%     + a3c * g *m3*cos(theta(1)+theta(2)+theta(3)) ...
%     + a1*((a2c*m2+a2*m3)*sin(theta(2)) + a3c*m3*sin(theta(2)+theta(3)))*thetadot(1)^2 ...
%     - 2*a2*a3c*m3*sin(theta(3))*thetadot(1)*thetadot(3) - 2 * a2*a3c*m3*sin(theta(3)) ...
%     *thetadot(2)*thetadot(3) - a2*a3c*m3*sin(theta(3))*thetadot(3)^2 ;
% B1 = (J2(3,3) + J3(3,3) + a2c^2*m2+a2^2*m3+a3c^2*m3+a1*(a2c*m2 + a2*m3)*cos(theta(2)) ...
%      + 2 * a2*a3c*m3*cos(theta(3)) + a1*a3c *m3 * cos(theta(2)+theta(3)));
% B2 = (J2(3,3) + J3(3,3) + a2c^2*m2 + a2^2*m3 + a3c^2*m3 + 2*a2*a3c*m3*cos(theta(3)));
% B3 = (J3(3,3) + a3c^2*m3+a2*a3c*m3*cos(theta(3)));
% 
% 
% G3 = a3c*m3*(g*cos(theta(1)+theta(2)+theta(3))+(a2*sin(theta(3))+a1*sin(theta(2)+theta(3))*thetadot(1)^2)+...
%     2*a2*sin(theta(3))*thetadot(1)*thetadot(2)+a2*sin(theta(3))*thetadot(2)^2);
% C1 = J3(3,3)+a3c^2*m3+a2*a3c*m3*cos(theta(3))+a1*a3c*cos(theta(2)+theta(3));
% C2 = J3(3,3)+a3c^2*m3+a2*a3c*m3*cos(theta(3));
% C3 = J3(3,3)+a3c^2*m3;
% 
% beta = [A1 A2 A3; B1 B2 B3; C1 C2 C3];
% delta = [u1 - G1;u2 - G2;u3 - G3];
% Gamma = beta\delta;
% 
% 
% thetadotdot = Gamma(1:3);
% thetadotdot(3) =0;
% thetadotdot(1) =0;





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%Involves the Drawing of the Robot
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function actionRecord = StoreAction(actionRecord,action)
actionRecord(:,end+1) = action;

function [actionRecord,action,done] = RetrieveAction(actionRecord)
action = actionRecord(:,1);
actionRecord = actionRecord(:,2:end);
done = isempty(actionRecord);

function [t,theta,thetadot] = Simulate(t,dt,theta,thetadot,u1,u2,u3,robot)            
[t,x] = ode45(@(t,x) GetXDot(t,x,u1,u2,u3,robot),[t t+dt],[theta;thetadot]);
x = x';
t = t(end);
[theta,thetadot] = XToState(x);

function xdot = GetXDot(t,x,u1,u2,u3,robot)
[theta,thetadot] = XToState(x);
[thetadot,thetadotdot] = GetRates(theta,thetadot,u1,u2,u3,robot);
xdot = [thetadot;thetadotdot];

function [theta,thetadot] = XToState(x)
theta = x(1:3,end);
thetadot = x(4:6,end);

function robotfig = DrawRobot(robotfig,robot,o_1in0,R_1in0,o_2in0,R_2in0,alpha)
eorange=[1,.30,0];
egrey=[0.4745,0.6471,0.9098];
eblue = [0,0,1];
if isempty(robotfig)
    % - joints
    p = robot.p_01in0;
    robotfig.joint1 = line(p(1),p(2),p(3),'color','k','marker','.','markersize',30);
    p = o_1in0+R_1in0*robot.p_12in1;
    robotfig.joint2 = line(p(1),p(2),p(3),'color','k','marker','.','markersize',30);
    p = o_2in0+R_2in0*robot.p_23in2;
    robotfig.joint3 = line(p(1),p(2),p(3),'color','k','marker','.','markersize',30);
    % - links
    robotfig.base = patch('Vertices',robot.base.p_in0','Faces',robot.faces,'FaceColor',egrey,...
                          'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                          'backfacelighting','reverselit','AmbientStrength',0.6);
    robotfig.link1 = patch('Vertices',robot.link1.p_in0','Faces',robot.faces,'FaceColor',eblue,...
                             'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                             'backfacelighting','reverselit','AmbientStrength',0.6);
	robotfig.link2 = patch('Vertices',robot.link2.p_in0','Faces',robot.faces,'FaceColor',eorange,...
                             'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                             'backfacelighting','reverselit','AmbientStrength',0.6);
	robotfig.link3 = patch('Vertices',robot.link3.p_in0','Faces',robot.faces,'FaceColor',eblue,...
                             'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                             'backfacelighting','reverselit','AmbientStrength',0.6);
else
     set(robotfig.link1,'vertices',robot.link1.p_in0');
     set(robotfig.link2,'vertices',robot.link2.p_in0');
     set(robotfig.link3,'vertices',robot.link3.p_in0');
    p = o_1in0+R_1in0*robot.p_12in1;
     set(robotfig.joint2,'xdata',p(1),'ydata',p(2),'zdata',p(3));
    p = o_2in0+R_2in0*robot.p_23in2;
     set(robotfig.joint3,'xdata',p(1),'ydata',p(2),'zdata',p(3));
end

function frame = DrawFrame(frame,o,R)
p = [o repmat(o,1,3)+R];
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',4);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',4);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',4);
else
     set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
     set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
     set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
end

function world = CreateFigure(robot,params)
% - clear the current figure
clf;
% - text (it's important this is in the back, so you can rotate the view
%         and other stuff!)
axes('position',[0 0 1 1]);
axis([0 1 0 1]);
hold on;
axis off;
fs = 10;
world.text.label=text(0.15,0.95,'view: frame 0','fontweight','bold','fontsize',fs);
world.text.time=text(0.05,0.1,sprintf('t = %6.2f / %6.2f\n',0,0),'fontsize',fs,'verticalalignment','top','fontname','monaco');
%world.text.teamname=text(0.05,0.04,params.teamname,'fontsize',fs,'verticalalignment','top','fontweight','bold');
world.text.torques=text(0.8,0.02,sprintf('u_1 = %6.1f\nu_2 = %6.1f\nu_3 = %6.1f',0,0,0),'fontsize',fs,'fontname','monaco','verticalalignment','bottom');
% - view from frame 0
axes('position',[0.05 0.05 .9 1]);
set(gcf,'renderer','opengl');
axis equal;
axis([-6.5 6.5 -15 15 -15 15]);
axis manual;
hold on;
view([90-20,20]);
box on;
 set(gca,'projection','perspective');
 set(gca,'clipping','on','clippingstyle','3dbox');
world.view0.robot = DrawRobot([],robot,zeros(3,1),eye(3),zeros(3,1),eye(3),0.6);
world.view0.frame0 = DrawFrame([],zeros(3,1),eye(3));
world.view0.frame1 = DrawFrame([],zeros(3,1),eye(3));
world.view0.frame2 = DrawFrame([],zeros(3,1),eye(3));
world.view0.frame3 = DrawFrame([],zeros(3,1),eye(3));
lighting gouraud
world.view0.light = light('position',zeros(3,1)','style','local');
% - make the figure respond to key commands
set(gcf,'KeyPressFcn',@onkeypress_nokeypad);

function world = UpdateFigure(world,robot,o_1in0,R_1in0,o_2in0,R_2in0,o_3in0,R_3in0,u1,u2,u3,t,tmax)
world.view0.robot = DrawRobot(world.view0.robot,robot,o_1in0,R_1in0,o_2in0,R_2in0);
world.view0.frame1 = DrawFrame(world.view0.frame1,o_1in0,R_1in0);
world.view0.frame2 = DrawFrame(world.view0.frame2,o_2in0,R_2in0);
world.view0.frame3 = DrawFrame(world.view0.frame3,o_3in0,R_3in0);
 set(world.view0.light,'position',([0;1;0])');
 set(world.text.time,'string',sprintf('t = %6.2f / %6.2f\n',t,tmax));
 set(world.text.torques,'string',sprintf('u_1 = %6.1f\nu_2 = %6.1f\nu_3 = %6.1f',u1,u2,u3));
drawnow

function onkeypress_nokeypad(src,event)
global action done
du = .5;
if event.Character == '1'
    action(1,1) = action(1,1)+du;
elseif event.Character == '!'
    action(1,1) = action(1,1)-du;
elseif event.Character == '2'
    action(2,1) = action(2,1)+du;
elseif event.Character == '@'
    action(2,1) = action(2,1)-du;
elseif event.Character == '3'
    action(3,1) = action(3,1)+du;
elseif event.Character == '#'
    action(3,1) = action(3,1)-du;
elseif event.Character == 'q'
    done = true;
end

                     

