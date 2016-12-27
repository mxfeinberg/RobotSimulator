s = tf('s');
phi = 1/(2.0708e-5 * s^2);
theta = 1/(2.0807e-5 * s^2);
psi = 1/(3.8860e-5 * s^2);
%Roll Axis
Kp_phi = 0.01042;
Kd_phi = 0.001226;
Ki_phi = 0.0447112;
D_phi = Kp_phi + Kd_phi*s + (Ki_phi/s);
%pitch Axis
Kp_theta = 0.01047;
Kd_theta = 0.001232;
Ki_theta = 0.044926;
D_theta = Kp_theta + Kd_theta*s + (Ki_theta/s);

%Yaw Axis
Kp_psi = 0.01047;
Kd_psi = 0.001232;
Ki_psi = 0.044926;
D_psi = Kp_psi + Kd_psi*s + (Ki_psi/s);

figure(1)
margin(D_phi * phi)
grid on
figure(2)
nyquist(D_phi * phi)
grid on

figure(3)
margin(D_theta * theta)
grid on
figure(4)
nyquist(1 + D_theta * theta)
grid on

figure(5)
margin(D_psi * psi)
grid on
figure(6)
nyquist(1 + D_psi * psi)
grid on

