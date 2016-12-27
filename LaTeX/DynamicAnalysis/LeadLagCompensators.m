s = tf('s');
K = 40;
G = K/(s*(s+4)*(s+1));
figure(1)
margin(G)


% Lead Initial
phi_max = 80%15+50+10%45-50+10%15+50+10;
alpha = (1-sind(phi_max))/(1+sind(phi_max));
omega = 16;
T = 1/sqrt(alpha)/omega;
Lead1 = (T*s+1)/(alpha*T*s+1);

% Lag Initial
alpha2 = 5;
T2 = 1/omega;
Lag1 = alpha2*(T2*s+1)/(alpha2*T2*s+1);

%Final Lead and Lag
Lead = ((s/1)+1)/((s/10)+1);
Lag = ((s/.4)+1)/((s/.051)+1);

figure(1)
margin(Lead1*Lag1*G)
grid on

figure(2)
margin(Lead*G*Lag)
grid on