clc;
close all;
clear all;
addpath('C:\Users\Aleksandr Prokohrov\Documents\ProkhorovAlex\UGA_INP\internship\MatLabProjects/functions/');
addpath('C:\Users\Aleksandr Prokohrov\Documents\ProkhorovAlex\UGA_INP\internship\MatLabProjects/synthesis_functions/');
addpath('C:\Users\Aleksandr Prokohrov\Documents\ProkhorovAlex\UGA_INP\internship\MatLabProjects/plot_figures_files/');

%%
config = readstruct("../sourceFiles/configuration.json");

xBodySize = config.Body.xSize;
yBodySize = config.Body.ySize;
zBodySize = config.Body.zSize;
bodyDensity = config.Body.density;

k01 = config.SD_1.spring;
c01 = config.SD_1.damping;
m01 = xBodySize*yBodySize*zBodySize*bodyDensity;

rRimSize = config.Rim.rRim;
hRimSize = config.Rim.hRim;
rimDensity = config.Rim.density;

k02 = config.SD_2.spring;
c02 = config.SD_2.damping;
m02 = 2*pi*rRimSize*hRimSize*rimDensity;

%%

% uncertanties definition as a 
% percentage variation of nominal values

% m = ureal('m', m, 'Percentage', [-40,40]);
delta_k = 1;
delta_c = 1;
delta_m = 1;
delta_k = 1-ureal('delta_k', delta_k, 'Percentage', [-50,50]);
delta_c = 1-ureal('delta_c', delta_c, 'Percentage', [-50,50]);
delta_m = 1-ureal('delta_m', delta_m, 'Percentage', [-50,50]);

k = (1+delta_k)*k02;
c = (1+delta_c)*c02;
m = (1+delta_m)*m02;
omega = sqrt(k.NominalValue/m.NominalValue);  % natural frequency
unc_max =  struct('delta_c2', 0.5, 'delta_k2', 0.5, 'delta_m2', 0.5);
unc_min =  struct('delta_c2', -0.5, 'delta_k2', -0.5, 'delta_m2', -0.5);

metrics = struct();
metrics_wc = struct();

%%

[A,B,C,D] = doubleSVDModel_MISO(m01, k01, c01, m, k, c);
u_system_miso = ss(A,B,C,D);         % uncertain system
u_system_miso_wc = worst_case_tf(u_system_miso);

[A,B,C,D] = doubleSVDModel(m01, k01, c01, m, k, c);

% uncertain system io and states definition
u_system = ss(A,B,C,D);         % uncertain system

u_system_miso.StateName = {'v2 (m/s)';'d2 (m)';'v1 (m/s)';'d1 (m)'};
u_system_miso.InputName = {'ui'};
u_system_miso.OutputName = {'x'};

figure('Name','OL_uncertain');

step(gridureal(u_system_miso, 50)); hold on; grid on;
step(u_system_miso.NominalValue, 'r');
legend('Uncertain Model', 'Nominal Model')
title('OL system');

Ts = 0.001;
sys_d = c2d(u_system_miso.NominalValue,Ts);

%% Reqirements
OS = 15;
[DampRatio, PM, Mt] = OverShoot(OS);
du = 0.5;
umax = 2*k01;
bmax = 0.1;

%% PID
% [S_pid, T_pid, SG_pid, KS_pid, PID_tf] = PID_synth(uss(system_siso), PM, du, umax, bmax);

[S_pid, T_pid, SG_pid, KS_pid, PID_tf, w_i, w_h, w_b, C] = PID_synth(u_system, PM, du, umax, bmax);
% [S_pid, T_pid, SG_pid, KS_pid, PID_tf] = PID_synth(u_system, 5, 8, 120, 230);
plot_step_response(S_pid, T_pid, SG_pid, KS_pid, 'PID CL system');

w=logspace(-3,3,500); %% to be adjusted
plot_sens_analysis(S_pid,T_pid,SG_pid,KS_pid,w) 

PID_d = c2d(PID_tf,Ts);

