% quadsim_tune_gains.m
%
% Initializer for quadsim.mdl.
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012

% Linearize 
[A B] = linearize_quadsim(P);
H = (s*eye(12)-A)\B;
H = minreal(H);
H = zpk(H);
kde=1; ktheta = 8; kpd=3; kdt=4; 
kphi=7; kda=2; kpsi=9; kdr=3;
-H(kpd,kdt);
H(ktheta,kde);
H(kphi,kda);
H(kpsi,kdr);

figure
Gkde2ktheta = PI_rateFeedback_TF(H(ktheta,kde), 0.11, 0.0075, 0.025); 
step(Gkde2ktheta, 20)
grid on;
title('G kde 2 ktheta Response');
xlabel('Time');
ylabel('Amplitude');

figure;
Gkda2phi = PI_rateFeedback_TF(H(kphi,kda), 0.11, 0.075, 0.025); 
step(Gkda2phi, 20)
grid on;
title('G kda 2 phi Response');
xlabel('Time');
ylabel('Amplitude');

figure;
Gkdt2kpd = PI_rateFeedback_TF(-H(kpd,kdt), 0.05, 0.0001, 0.07); 
step(Gkdt2kpd, 20)
grid on;
title('G kdt 2 kpd Response');
xlabel('Time');
ylabel('Amplitude');

figure;
Gkdr2kpsi = PI_rateFeedback_TF(H(kpsi,kdr), 0.11, 0.002, 0.07); 
step(Gkdr2kpsi, 20) 
grid on;
title('G kdr 2 kpsi Response');
xlabel('Time');
ylabel('Amplitude');

figure;
GxhorVel = PI_rateFeedback_TF( Gkde2ktheta * G_pitch2Vhx, -0.035, -0.025, 0);
step(GxhorVel, 20);
grid on;
title('G xhor Vel Response');
xlabel('Time');
ylabel('Amplitude');

figure;
GyhorVel =PI_rateFeedback_TF(Gkda2phi*G_roll2Vhx, 0.05, 0.025, 0);
step(GyhorVel, 20);
grid on;
title('G yhor Vel Response');
xlabel('Time');
ylabel('Amplitude');