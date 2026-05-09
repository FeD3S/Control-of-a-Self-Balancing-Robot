%% General parameters and conversion gains

% controller sampling time
Ts = 1e-2;

% gravity acc [m/s^2]
g = 9.81;

% conversion gains
rpm2rads = 2*pi/60;        % [rpm] -> [rad/s]
rads2rpm = 60/2/pi;        % [rad/s] -> [rpm]
rpm2degs = 360/60;         % [rpm] -> [deg/s]
degs2rpm = 60/360;         % [deg/s] -> [rpm]
deg2rad = pi/180;          % [deg] -> [rad]
rad2deg = 180/pi;          % [rad] -> [deg]
g2ms2 = g;                 % [acc_g] -> [m/s^2]
ms22g = 1/g;               % [m/s^2] -> [acc_g]
ozin2Nm = 0.706e-2;        % [oz*inch] -> [N*m]

% robot initial condition
x0 = [ ...
    0, ...                 % gam(0)
    5*deg2rad, ...         % th(0)
    0, ...                 % dot_gam(0)
    0];                    % dot_th(0)

%% DC motor data
% motor id: brushed DC gearmotor Pololu 30:1 37Dx68L mm

% electromechanical params
mot.UN = 12;
mot.taus = 110/30 * ozin2Nm;
mot.Is = 5;
mot.w0 = 350 * 30 * rpm2rads;
mot.I0 = 0.3;

mot.R = mot.UN/mot.Is;
mot.L = NaN;
mot.Kt = mot.taus/mot.Is;
mot.Ke = (mot.UN - mot.R*mot.I0)/(mot.w0);
mot.eta = NaN;
mot.PN = NaN;
mot.IN = NaN;
mot.tauN = NaN;

% dimensions
mot.rot.h = 30.7e-3;
mot.rot.r = 0.9 * 17e-3;
mot.stat.h = 68.1e-3;
mot.stat.r = 17e-3;

% center of mass (CoM) position
mot.rot.xb = 0;
mot.rot.yb = 42.7e-3;
mot.rot.zb = -7e-3;
mot.stat.xb = 0;
mot.stat.yb = 52.1e-3;
mot.stat.zb = -7e-3;

% mass
mot.m = 0.215;
mot.rot.m = 0.35 * mot.m;
mot.stat.m = mot.m - mot.rot.m;

% moment of inertias (MoI)
mot.rot.Ixx = mot.rot.m/12 * (3*mot.rot.r^2 + mot.rot.h^2);
mot.rot.Iyy = mot.rot.m/2 * mot.rot.r^2;
mot.rot.Izz = mot.rot.Ixx;

mot.stat.Ixx = mot.stat.m/12 * (3*mot.stat.r^2 + mot.stat.h^2);
mot.stat.Iyy = mot.stat.m/2 * mot.stat.r^2;
mot.stat.Izz = mot.stat.Ixx;

% viscous friction coeff (motor side)
mot.B = mot.Kt * mot.I0 / mot.w0;

%% Gearbox data
gbox.N = 30;
gbox.B = 0.025;

%% Battery data
batt.UN = 11.1;
batt.w = 136e-3;
batt.h = 26e-3;
batt.d = 44e-3;

batt.xb = 0;
batt.yb = 0;
batt.zb = 44e-3;

batt.m = 0.320;

batt.Ixx = batt.m/12 * (batt.w^2 + batt.h^2);
batt.Iyy = batt.m/12 * (batt.d^2 + batt.h^2);
batt.Izz = batt.m/12 * (batt.w^2 + batt.d^2);

%% H-bridge PWM voltage driver data
drv.Vbus = batt.UN;
drv.pwm.bits = 8;
drv.pwm.levels = 2^drv.pwm.bits;
drv.dutymax = drv.pwm.levels - 1;
drv.duty2V = drv.Vbus / drv.dutymax;
drv.V2duty = drv.dutymax / drv.Vbus;

%% Wheel data
wheel.h = 26e-3;
wheel.r = 68e-3 / 2;

wheel.xb = 0;
wheel.yb = 100e-3;
wheel.zb = 0;

wheel.m = 50e-3;

wheel.Ixx = wheel.m/12 * (3*wheel.r^2 + wheel.h^2);
wheel.Iyy = wheel.m/2 * wheel.r^2;
wheel.Izz = wheel.Ixx;

wheel.B = 0.0015;

%% Chassis data
chassis.w = 160e-3;
chassis.h = 119e-3;
chassis.d = 80e-3;

chassis.xb = 0;
chassis.yb = 0;
chassis.zb = 80e-3;

chassis.m = 0.456;

chassis.Ixx = chassis.m/12 * (chassis.w^2 + chassis.h^2);
chassis.Iyy = chassis.m/12 * (chassis.d^2 + chassis.h^2);
chassis.Izz = chassis.m/12 * (chassis.w^2 + chassis.d^2);

%% Body data
body.m = chassis.m + batt.m + 2*mot.stat.m;

body.xb = 0;
body.yb = 0;
body.zb = (1/body.m) * (chassis.m*chassis.zb + ...
                        batt.m*batt.zb + 2*mot.stat.m*mot.stat.zb);

body.Ixx = chassis.Ixx + chassis.m*(body.zb - chassis.zb)^2 + ...
           batt.Ixx + batt.m*(body.zb - batt.zb)^2 + ...
           2*mot.stat.Ixx + ...
           2*mot.stat.m*(mot.stat.yb^2 + (body.zb - mot.stat.zb)^2);

body.Iyy = chassis.Iyy + chassis.m*(body.zb - chassis.zb)^2 + ...
           batt.Iyy + batt.m*(body.zb - batt.zb)^2 + ...
           2*mot.stat.Iyy + ...
           2*mot.stat.m*(body.zb - mot.stat.zb)^2;

body.Izz = chassis.Izz + batt.Izz + ...
           2*mot.stat.Izz + 2*mot.stat.m*mot.stat.yb^2;

%% Sensors data − Hall-effect encoder
sens.enc.ppr = 16 * 4;
sens.enc.pulse2deg = 360 / sens.enc.ppr;
sens.enc.pulse2rad = 2*pi / sens.enc.ppr;
sens.enc.deg2pulse = sens.enc.ppr / 360;
sens.enc.rad2pulse = sens.enc.ppr / (2*pi);

%% Sensors data − MPU6050 (accelerometer + gyro)
sens.mpu.xb = 0;
sens.mpu.yb = 0;
sens.mpu.zb = 13.5e-3;

% Accelerometer
sens.mpu.acc.bits = 16;
sens.mpu.acc.fs_g = 16;
sens.mpu.acc.fs = sens.mpu.acc.fs_g * g2ms2;
sens.mpu.acc.g2LSB = floor(2^(sens.mpu.acc.bits - 1) / sens.mpu.acc.fs_g);
sens.mpu.acc.ms22LSB = sens.mpu.acc.g2LSB * ms22g;
sens.mpu.acc.LSB2g = sens.mpu.acc.fs_g / 2^(sens.mpu.acc.bits - 1);
sens.mpu.acc.LSB2ms2 = sens.mpu.acc.LSB2g * g2ms2;
sens.mpu.acc.bw = 94;
sens.mpu.acc.noisestd = 400e-6 * sqrt(100);
sens.mpu.acc.noisevar = sens.mpu.acc.noisestd^2;

% Gyroscope
sens.mpu.gyro.bits = 16;
sens.mpu.gyro.fs_degs = 250;
sens.mpu.gyro.fs = sens.mpu.gyro.fs_degs * deg2rad;
sens.mpu.gyro.degs2LSB = floor(2^(sens.mpu.gyro.bits - 1) / sens.mpu.gyro.fs_degs);
sens.mpu.gyro.rads2LSB = sens.mpu.gyro.degs2LSB * rad2deg;
sens.mpu.gyro.LSB2degs = sens.mpu.gyro.fs_degs / 2^(sens.mpu.gyro.bits - 1);
sens.mpu.gyro.LSB2rads = sens.mpu.gyro.LSB2degs * deg2rad;
sens.mpu.gyro.bw = 98;
sens.mpu.gyro.noisestd = 5e-3 * sqrt(100);
sens.mpu.gyro.noisevar = sens.mpu.acc.noisestd^2;
