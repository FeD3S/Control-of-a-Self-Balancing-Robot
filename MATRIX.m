
M11 = 2 * wheel.Iyy + 2 * gbox.N^2 * mot.rot.Iyy + ( body.m + 2 * wheel.m + 2 * mot.rot.m ) * wheel.r^2; % DOUBTS ON WHEEL.R
M12 = 2 * gbox.N * ( 1 - gbox.N ) * mot.rot.Iyy + ( body.m * body.zb + 2 * mot.rot.m * mot.rot.zb ) * wheel.r; % l = body.zb
M21 = M12;
M22 = body.Iyy + 2 * ( 1 - gbox.N )^2 * mot.rot.Iyy + body.m * body.zb^2 + 2 * mot.rot.m * mot.rot.zb^2;

M = [ M11, M12; M21, M22];

%% g vector

g1 = 0;
g2 = @(q) -(body.m * body.zb + 2 * mot.rot.m * mot.rot.zb) * g * sin(q);

G11 = 0;
G12 = 0;
G21 = 0;
G22 = -(body.m * body.zb + 2 * mot.rot.m * mot.rot.zb) * g;
G = [[G11, G12];[G21,G22]];

%% C matrix

C12 = @(q, q_dot) -(body.m * body.zb + 2 * mot.rot.m * mot.rot.zb ) * wheel.r * sin(q) * q_dot;

%% F matrix

Fv11 = 2 * ( gbox.B + wheel.B );
Fv12 = -2 * gbox.B;
Fv21 = Fv12;
Fv22 = - Fv12;

Fv = [ Fv11, Fv12; Fv21, Fv22];

%% F prime matrix
Fv_prime = Fv + 2*gbox.N^2*mot.Kt*mot.Ke/mot.R*[[1 -1];[-1 1]] ;


%%

ua2tau1 = ( 2 * gbox.N * mot.Kt / mot.R) * [1, -1]';