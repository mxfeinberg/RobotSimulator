
s = tf('s');
G = 40/(s*(s+2));
phi_max = 32+10;
alpha = (1-sind(phi_max))/(1+sind(phi_max));
wMax = 10;
T = 1/(wMax * sqrt(alpha));
%Lead Compensator
DLead1 = (T*s+1)/(alpha*T*s+1);
figure (1)
margin(DLead1*G)
grid on
figure (2)
margin(DLead2*G)
grid on

figure(3)
nyquist(G)
grid on

figure(4)
margin(G)
grid on

