ssc.A = [[zeros(2,2), eye(2)];[-M^-1*G, -M^-1*Fv_prime]];
ssc.B = 2*gbox.N*mot.Kt/mot.R*[zeros(2,2);M^-1]*[1;-1];
ssc.C = [1, 0, 0, 0];

ssc.sysc = ss(ssc.A,ssc.B,ssc.C,0);
ssc.sysd = c2d(ssc.sysc, 1e-2, 'zoh');

ssc.phi = ssc.sysd.A;
ssc.gamma = ssc.sysd.B;
ssc.H = ssc.sysd.C;

theta_max = pi/360;   
gamma_max = pi / 36;               
u_max     = 1;

theta_bar = pi/360;
gamma_bar = pi/18;
u_bar = 1;
r = u_bar^-2;
rho1 = 500;
rho2 = 5000;
Q = diag([gamma_bar^-2,theta_bar^-2,0,0]);

sol = [[ssc.phi-eye(4) ssc.gamma];[ssc.H 0]]\[[0;0;0;0];1];
ssc.Nx = sol(1:4);
ssc.Nu = sol(5);

ssc.K1 = dlqr(ssc.phi,ssc.gamma,Q,r*rho1);
ssc.K2 = dlqr(ssc.phi,ssc.gamma,Q,r*rho2);

%% robust tracking

ssc_aug.phie = [[1, ssc.H];[[0;0;0;0],ssc.phi]];
ssc_aug.gammae = [0;ssc.gamma];
q1 = 0.1;
q2 = 1;
Q1 = diag([q1,gamma_bar^-2,theta_bar^-2,0,0]);
Q2 = diag([q2,gamma_bar^-2,theta_bar^-2,0,0]);

ssc_aug.Nx = ssc.Nx;
ssc_aug.Nu = ssc.Nu;

ssc_aug.K1e = dlqr(ssc_aug.phie,ssc_aug.gammae, Q1, r*rho1);
ssc_aug.Ki1 = ssc_aug.K1e(1);
ssc_aug.K1 = ssc_aug.K1e(2:end);

ssc_aug.K2e = dlqr(ssc_aug.phie,ssc_aug.gammae, Q1, r*rho2);
ssc_aug.Ki2 = ssc_aug.K2e(1);
ssc_aug.K2 = ssc_aug.K2e(2:end);

ssc_aug.K3e = dlqr(ssc_aug.phie,ssc_aug.gammae, Q2, r*rho1);
ssc_aug.Ki3 = ssc_aug.K3e(1);
ssc_aug.K3 = ssc_aug.K3e(2:end);

ssc_aug.K4e = dlqr(ssc_aug.phie,ssc_aug.gammae, Q2, r*rho2);
ssc_aug.Ki4 = ssc_aug.K4e(1);
ssc_aug.K4 = ssc_aug.K4e(2:end);