%% Intro
% This document verifies if the  directional Fixed-body robot's GIE
% maxes out by the total mass of the robot. Analyzed a planar, 5-DoF,
% Fixed, serial robot on xy-plane. The first 3-DoF consist of px, py, rz of
% the torso(3), followed by 2-DoF of rz's of hip and knee. 

%% Init
clear; clc; 
addpath(genpath(pwd))

% Import Dynamics 
load_dyn = load('robot_dynamics.mat');  % from gen_Fixed_leg.m
robot_dyn = load_dyn.robot;

% Import Numerical Values of L, m, I
params_value = import_robot_params('value');
L = params_value.L;
m = params_value.m;
I = params_value.I;

%% Variables and Parameters
% angles
sz_q = 200;                  % array size
del = 0.05;
q1 = linspace(0-del, -pi/2+del, sz_q);     % hip angle
q2 = -pi - 2*q1;             % knee angle
q = [q1; q2];   

% gears
alpha_inertia_scaler = 0.1;
referece_link_idx = 1;
const_reflected_inertia = alpha_inertia_scaler *1/3* m(referece_link_idx) * L(referece_link_idx)^2;
N_ = 50;  % 6, 50
Irot_ = const_reflected_inertia ./ (N_.^2);
gear = [Irot_; Irot_; N_; N_];

% efficiency
sz_eff = 300;                % array size 
n_fwd1 = linspace(0.5, 1, sz_eff);     % forward eff array
n_fwd2 = n_fwd1;
n_bwd1 = fcn_f2b_eff(n_fwd1, N_);     % backward eff array
n_bwd2 = fcn_f2b_eff(n_fwd2, N_);
% n_bwd1 = linspace(0.5, 1, sz_eff);
% n_bwd2 = n_bwd1;
eff = [n_bwd1;n_bwd2;n_fwd1;n_fwd2];    

% intercept
unit_vec = [0; 1];          % directional task-space inertia's direction
origin = [0;0];
scale = 1;

%% Figure
figure()
sgtitle({'Vertical Mass vs Hip Angle, Fixed 2 DoF',['Non-dim Rotor inertia scale = ',num2str(alpha_inertia_scaler)]})
limy = [0 500];
limx = [ -90 0 ];
n_subplot = 2;

%% subplot
subplot(n_subplot ,1,1)
scenario = 'fwd';
drive = 'ser';
vertical_mass_fwd = zeros(sz_q, sz_eff);

for idx_q = 1:sz_q
    for idx_eff = 1:sz_eff
        q_ = q(:, idx_q);
        eff_ = eff(:,idx_eff);
        Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
        vertical_mass_fwd(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');
    end
end

for idx_eff = 1:sz_eff
    color = 0.7*(1- 1*idx_eff/sz_eff) ;
    plot(q1*180/pi, vertical_mass_fwd(:, idx_eff),'color', ones(3,1)*color)
    hold on
end

title(['FWD, N = ', num2str(N_)])
xlabel('(stretched)<--- hip angle, deg --->(bent)')
ylabel('vertical mass, kg')
xlim(limx)
ylim(limy)
grid minor

%% subplot
subplot(n_subplot ,1,2)
scenario = 'bwd';
drive = 'ser';
vertical_mass_bwd = zeros(sz_q, sz_eff);

for idx_q = 1:sz_q
    for idx_eff = 1:sz_eff
        q_ = q(:, idx_q);
        eff_ = eff(:,idx_eff);
        Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
        vertical_mass_bwd(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');        
    end
end

for idx_eff = 1:sz_eff
    color = 0.7*(1- 1*idx_eff/sz_eff) ;
    plot(q1*180/pi, vertical_mass_bwd(:, idx_eff),'color', ones(1,3)*color)
    hold on
end

title(['BWD, N = ', num2str(N_)])
xlabel('(stretched)<--- hip angle, deg --->(bent)')
ylabel('vertical mass, kg')
xlim(limx)
ylim(limy)
grid minor


set(gcf,'Position',[100 100 400 450])
