%% Intro
% This document verifies if the  directional floating-body robot's GIE
% maxes out by the total mass of the robot. Analyzed a planar, 5-DoF,
% floating, serial robot on xy-plane. The first 3-DoF consist of px, py, rz of
% the torso(3), followed by 2-DoF of rz's of hip and knee. 

%% Init
clear; clc; close all;
addpath(genpath(pwd))

% Import Dynamics 
load_dyn = load('robot_dynamics.mat');  % from gen_floating_leg.m
robot_dyn = load_dyn.robot;

% Import Numerical Values of L, m, I
params_value = import_robot_params('value-light');
L = params_value.L;
m = params_value.m;
I = params_value.I;

%% Prep
% sgtitle('Vertical Mass vs Hip Angle, Floating 2 DoF')
% limy = [0 sum(m)*1.1];
% limx = [ -90 0 ];

% reflected inertia 
alpha_inertia_scaler = 1;
const_reflected_inertia = alpha_inertia_scaler *1/3* m(1) * L(1)^2;
N_ = [3,12,50,100];
Irot_ = const_reflected_inertia ./ (N_.^2);
sz_N = length(N_);
gear_ = [Irot_; Irot_; N_; N_];          

% pose
sz_q = 5;
q1_ = linspace(-0.2, -pi/2+0.2, sz_q);
q2_ = -pi - 2*q1_;
% q1_sample = -pi/2.5;
% q2_sample = -pi - 2*q1_sample;
% q_sample = [q1_sample, q2_sample];
q_ = [q1_;q2_];

% efficiency
eff_ideal = ones(4,1);
sz_eff = 20;
eff_N = [.95, .90, .85, .80];


%% Figure
figure(1)
sgtitle(['ATIE with Identical Reflected Inertia, nondim-scale: ', num2str(alpha_inertia_scaler)])
origin = [0;0];
scale = 1;
limx = [-10, 10];
limy = [-20, 20];
unit_vec = [0;1];
vertical_mass = zeros(sz_N, 3, sz_q);


for idx_N = 1:sz_N
    for idx_q = 1:sz_q
        eff_ = [eff_N(idx_N); eff_N(idx_N); fcn_f2b_eff(eff_N(idx_N), gear_(3, idx_N)); fcn_f2b_eff(eff_N(idx_N), gear_(3, idx_N))];
        Zi = calcAGIE(q_(:, idx_q), L, m, I, gear_(:, idx_N), eff_ideal, 'idl','par');
        Zb = calcAGIE(q_(:, idx_q), L, m, I, gear_(:, idx_N), eff_, 'bwd','par');
        Zf = calcAGIE(q_(:, idx_q), L, m, I, gear_(:, idx_N), eff_, 'fwd','par');
        massi = intercept_ellip(Zi, unit_vec, 'norm');
        massb = intercept_ellip(Zb, unit_vec, 'norm');
        massf = intercept_ellip(Zf, unit_vec, 'norm');
        vertical_mass(idx_N, 1:3, idx_q) = [massi, massb, massf];
        
        subplot(sz_q, sz_N+1, (sz_N+1)*(idx_q-1) + idx_N)
        plot_ellip_map(Zi, origin, scale, 'color','k');
        hold on
        plot_ellip_map(Zb, origin, scale, 'color','r');
        plot_ellip_map(Zf, origin, scale, 'color','b') ;
    end
end

%%
for idx_N = 1:sz_N
    for idx_q = 1:sz_q
        subplot(sz_q, sz_N+1, (sz_N+1)*(idx_q-1) + idx_N)
        xlabel('mass, kg')
        ylabel('mass, kg')
        title(['N = ', num2str(N_(idx_N))])
        xlim(limx)
        ylim(limy)
        grid on
    end
end

for idx_q = 1: sz_q
    subplot(sz_q , sz_N+1, (idx_q)*(sz_N+1))
    plot(1:4, vertical_mass(:, 1, idx_q), 'color','k','Marker', 's')
    hold on
    plot(1:4, vertical_mass(:, 2, idx_q), 'color','r','Marker', 's')
    plot(1:4, vertical_mass(:, 3, idx_q), 'color','b','Marker', 's')
    legend('ideal', 'bwd', 'fwd','Location','southeast')
    name = {'N=3', 'N=12', 'N=50', 'N=100'};
    grid on
    title(['hip angle = ', num2str(floor(q_(1, idx_q)*180/pi)), ' deg'])
    
    xticks(1:4)
    xticklabels(name)    
    ymax = max(max(vertical_mass(:,:,idx_q)))*1.1;
    ylim([0, ymax])
    xlim([.5, 4.5])
end
%%
set(gcf,'Position',[0 0 1500 1400])


    

% %% Variables and Parameters
% % angles
% sz_q = 200;                  % array size
% qx = zeros(1, sz_q);        % torso coordinate 1 (set to zero, trivial)
% qy = qx;                    % torso coordinate 2 (set to zero, trivial)
% q1 = qx;                    % torso coordinate 3 (set to zero, trivial)
% del = .1;
% q2 = linspace(0-del, -pi/2+del, sz_q);     % hip angle
% q3 = pi - 2*q2;             % knee angle
% % q3 = -pi/2 - q2;             % knee angle
% q = [qx; qy; q1; q2; q3];   
% 
% % gears
% Irot1 = 0.0023/6^2;                % Inertia of Rotor
% Irot2 = Irot1;
% N1 = 50;                     % Gearing Ratio
% N2 = N1;
% gear = [Irot1; Irot2; N1; N2];          
% 
% % efficiency
% sz_eff = 100;                % array size 
% n_fwd1 = linspace(0.5, 1, sz_eff);     % forward eff array
% n_fwd2 = n_fwd1;
% n_bwd1 = fcn_f2b_eff(n_fwd1, N1);     % backward eff array
% n_bwd2 = fcn_f2b_eff(n_fwd2, N2);
% % n_bwd1 = linspace(0.5, 1, sz_eff);
% % n_bwd2 = n_bwd1;
% eff = [n_bwd1;n_bwd2;n_fwd1;n_fwd2];    
% 
% % intercept
% unit_vec = [0; 1];          % directional task-space inertia's direction
% origin = [0;0];
% scale = 1;
% %% subplot
% subplot(3,1,1)
% scenario = 'fwd';
% drive = 'ser';
% vertical_mass = zeros(sz_q, sz_eff);
% 
% for idx_q = 1:sz_q
%     for idx_eff = 1:sz_eff
%         q_ = q(:, idx_q);
%         eff_ = eff(:,idx_eff);
%         Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
%         vertical_mass(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');
%     end
% end
% 
% for idx_eff = 1:sz_eff
%     color = 0.7*(1- 1*idx_eff/sz_eff) ;
%     plot(q2*180/pi, vertical_mass(:, idx_eff),'color', ones(3,1)*color)
%     hold on
% end
% 
% title(['FWD, N = ', num2str(N1)])
% xlabel('(stretched)<--- hip angle, deg --->(bent)')
% ylabel('vertical mass, kg')
% xlim(limx)
% ylim(limy)
% grid minor
% 
% subplot(3,1,3)
% eff_sample = [0.75, 0.75, fcn_f2b_eff(.75, N1), fcn_f2b_eff(.75, N2)];
% Zf = calcAGIE(q_sample, L, m, I, gear, eff_sample, scenario, drive);
% 
% plot_ellip(gca, Zf, origin, scale);
% hold on
% 
% %% subplot
% subplot(3,1,2)
% scenario = 'bwd';
% drive = 'ser';
% vertical_mass = zeros(sz_q, sz_eff);
% 
% for idx_q = 1:sz_q
%     for idx_eff = 1:sz_eff
%         q_ = q(:, idx_q);
%         eff_ = eff(:,idx_eff);
%         Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
%         vertical_mass(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');        
%     end
% end
% 
% for idx_eff = 1:sz_eff
%     color = 0.7*(1- 1*idx_eff/sz_eff) ;
%     plot(q2*180/pi, vertical_mass(:, idx_eff),'color', ones(1,3)*color)
%     hold on
% end
% 
% title(['BWD, N = ', num2str(N1)])
% xlabel('(stretched)<--- hip angle, deg --->(bent)')
% ylabel('vertical mass, kg')
% xlim(limx)
% ylim(limy)
% grid minor
% 
% subplot(3,1,3)
% qx_sample = 0;
% qy_sample = 0;
% q1_sample = 0;
% q2_sample = -pi/3;
% q3_sample = -pi/3;
% q_sample = [qx_sample, qy_sample, q1_sample, q2_sample, q3_sample];
% eff_sample = [fcn_f2b_eff(.6, N1), fcn_f2b_eff(.6, N2), 0.6, 0.6];
% Zb = calcAGIE(q_sample, L, m, I, gear, eff_sample, scenario, drive);
% 
% plot_ellip(gca, Zb, origin, scale);
% 
