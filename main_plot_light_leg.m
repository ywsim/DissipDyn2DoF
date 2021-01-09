%% Intro
% This document verifies if the  directional floating-body robot's GIE
% maxes out by the total mass of the robot. Analyzed a planar, 5-DoF,
% floating, serial robot on xy-plane. The first 3-DoF consist of px, py, rz of
% the torso(3), followed by 2-DoF of rz's of hip and knee. 

%% Init
clear; clc; 
addpath(genpath(pwd))

% Import Dynamics 
load_dyn = load('robot_dynamics.mat');  % from gen_floating_leg.m
robot_dyn = load_dyn.robot;

% Import Numerical Values of L, m, I
params_value = import_robot_params('value-light');
L = params_value.L;
m = params_value.m;
I = params_value.I;

%% Figure
figure()
sgtitle('vertical mass vs hip angle, LightWeight, darker = higher efficiency')
limy = [0 14];
limx = [ -90 0 ];

%% Variables and Parameters
% angles
sz_q = 200;                  % array size
qx = zeros(1, sz_q);        % torso coordinate 1 (set to zero, trivial)
qy = qx;                    % torso coordinate 2 (set to zero, trivial)
q1 = qx;                    % torso coordinate 3 (set to zero, trivial)
q2 = linspace(0, -pi/2, sz_q);     % hip angle
q3 = pi - 2*q2;             % knee angle
q = [qx; qy; q1; q2; q3];   

% gears
Irot1 = 0.002;                % Inertia of Rotor
Irot2 = Irot1;
N1 = 10;                     % Gearing Ratio
N2 = N1;
gear = [Irot1; Irot2; N1; N2];          

% efficiency
sz_eff = 100;                % array size 
n_fwd1 = linspace(0.5, 1, sz_eff);     % forward eff array
n_fwd2 = n_fwd1;
n_bwd1 = fcn_f2b_eff(n_fwd1, N1);     % backward eff array
n_bwd2 = fcn_f2b_eff(n_fwd2, N2);
eff = [n_bwd1;n_fwd1;n_bwd2;n_fwd2];    

% intercept
unit_vec = [0; 1];          % directional task-space inertia's direction

%% subplot
subplot(2,2,1)
scenario = 'fwd';
drive = 'ser';
vertical_mass = zeros(sz_q, sz_eff);

for idx_q = 1:sz_q
    for idx_eff = 1:sz_eff
        q_ = q(:, idx_q);
        eff_ = eff(:,idx_eff);
        Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
        vertical_mass(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');
    end
end

for idx_eff = 1:sz_eff
    color = 1-1.0*idx_eff/sz_eff;
    plot(q2*180/pi, vertical_mass(:, idx_eff),'color', ones(3,1)*color)
    hold on
end

title('FWD, N = 10  ')
xlabel('(stretched)<--- hip angle, deg --->(bent)')
ylabel('vertical mass, kg')
xlim(limx)
ylim(limy)
grid minor

%% subplot
subplot(2,2,2)
scenario = 'bwd';
drive = 'ser';
vertical_mass = zeros(sz_q, sz_eff);

for idx_q = 1:sz_q
    for idx_eff = 1:sz_eff
        q_ = q(:, idx_q);
        eff_ = eff(:,idx_eff);
        Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
        vertical_mass(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');        
    end
end

for idx_eff = 1:sz_eff
    color = 1- 1*idx_eff/sz_eff;
    plot(q2*180/pi, vertical_mass(:, idx_eff),'color', ones(1,3)*color)
    hold on
end

title('BWD, N = 10  ')
xlabel('(stretched)<--- hip angle, deg --->(bent)')
ylabel('vertical mass, kg')
xlim(limx)
ylim(limy)
grid minor

%% Variables and Parameters
% angles
sz_q = 200;                  % array size
qx = zeros(1, sz_q);        % torso coordinate 1 (set to zero, trivial)
qy = qx;                    % torso coordinate 2 (set to zero, trivial)
q1 = qx;                    % torso coordinate 3 (set to zero, trivial)
q2 = linspace(0, -pi/2, sz_q);     % hip angle
q3 = pi - 2*q2;             % knee angle
q = [qx; qy; q1; q2; q3];   

% gears
Irot1 = 0.002;                % Inertia of Rotor
Irot2 = Irot1;
N1 = 100;                     % Gearing Ratio
N2 = N1;
gear = [Irot1; Irot2; N1; N2];          

% efficiency
sz_eff = 100;                % array size 
n_fwd1 = linspace(0.6, 1, sz_eff);     % forward eff array
n_fwd2 = n_fwd1;
n_bwd1 = fcn_f2b_eff(n_fwd1, N1);     % backward eff array
n_bwd2 = fcn_f2b_eff(n_fwd2, N2);
eff = [n_bwd1;n_fwd1;n_bwd2;n_fwd2];    

% intercept
unit_vec = [0; 1];          % directional task-space inertia's direction

%% subplot
subplot(2,2,3)
scenario = 'fwd';
drive = 'ser';
vertical_mass = zeros(sz_q, sz_eff);

for idx_q = 1:sz_q
    for idx_eff = 1:sz_eff
        q_ = q(:, idx_q);
        eff_ = eff(:,idx_eff);
        Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
        vertical_mass(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');      
    end
end

for idx_eff = 1:sz_eff
    color = 1-1.0*idx_eff/sz_eff;
    plot(q2*180/pi, vertical_mass(:, idx_eff),'color', ones(3,1)*color)
    hold on
end

title('FWD, N = 100')
xlabel('(stretched)<--- hip angle, deg --->(bent)')
ylabel('vertical mass, kg')
xlim(limx)
ylim(limy)
grid minor

%% subplot
subplot(2,2,4)
scenario = 'bwd';
drive = 'ser';
vertical_mass = zeros(sz_q, sz_eff);

for idx_q = 1:sz_q
    for idx_eff = 1:sz_eff
        q_ = q(:, idx_q);
        eff_ = eff(:,idx_eff);
        Z = calcAGIE(q_, L, m, I, gear, eff_, scenario, drive);
        vertical_mass(idx_q, idx_eff) = intercept_ellip(Z, unit_vec, 'norm');
    end
end

for idx_eff = 1:sz_eff
    color = 1- 1*idx_eff/sz_eff;
    plot(q2*180/pi, vertical_mass(:, idx_eff),'color', ones(1,3)*color)
    hold on
end

title('BWD, N = 100  ')
xlabel('(stretched)<--- hip angle, deg --->(bent)')
ylabel('vertical mass, kg')
xlim(limx)
ylim(limy)
grid minor

%% %%
% origin = [0;0];
% scale = 1;
% figure(1);
% hold on
% plot_ellip(gca, iner, origin, scale);
% 
% 
% 
% daspect(ones(3,1))
% 
% 
% % figure(2)
% plot_ellip2(gca, iner, origin, scale);
% 
% legend('sq', 'map')
% %% Task-Space Inertia felt at the Tip
% sz_q = 50;
% q2_ = linspace(-0.0001, -pi/2+0.0001, sz_q);
% q1_ = zeros(size(q2_));
% q3_ = pi - 2*q2_;
% 
% q_ = [ zeros(2, sz_q); q1_; q2_; q3_];
% N = 100;
% I_rot = 0.002;
% H_mot = diag(ones(2,1)*N^2*I_rot);
% 
% iner =      inertia(q_, L_, m_, I_);
% % iner_refl = inertia_reflected(q_, L_, m_, I_, H_mot);
% 
% unit_vector = [0; 1];
% y_intercept = zeros(size(q2_));
% 
% figure(2)
% hold on
% ca = gca;
% for idx_q = 1:sz_q
%     iner =      inertia(q_(:,idx_q), L_, m_, I_);
%     plot_ellip(ca, iner, [0;0], 1);
%     intercept = intercept_ellip(iner, unit_vector);
%     y_intercept(idx_q) = norm(intercept);
% %     pause(0.2)
% end
% 
% figure()
% plot(q2_*180/pi, y_intercept)
% xlabel('hip angle in xy-plane, deg')
% ylabel('vertical inertia, kg')
% title('vertical inertia felt at tip')