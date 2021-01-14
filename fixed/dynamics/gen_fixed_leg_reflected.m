function robot = gen_fixed_leg_reflected()
%% Init
addpath(genpath(pwd))
clear;clc;

robot.name = '2 DOF 2-Link Planar Monoped';
robot.base = 'fixed';
robot.dimensions = 'planar';
robot.flagFloat = false;
robot.transmission = 'ser';

robot.parent = [0 1 0 1 ];  

robot.nBase = 0;
robot.nLimbs = 2;
robot.nTransmission = 2;
robot.nd = (robot.nBase + robot.nLimbs + robot.nTransmission);% number of DoF
robot.NB = robot.nd;    % number of body

%% Import Params
params_ = import_robot_params('symbolic');

%% Rigid Body Params
mass_ = [params_.m; 0; 0];

com_ = [    params_.L(1)/2 0 0;
            params_.L(2)/2 0 0
            zeros(2,3)];

iner_ = [   0 0 params_.I(1);
            0 0 params_.I(2);
            0 0 params_.Irot(1);
            0 0 params_.Irot(2);];
     
jtype_ = {'Rz','Rz', 'Rz', 'Rz'};

jpos_ = [   zeros(1,3);
            params_.L(1) 0 0;
            zeros(1,3);
            params_.L(1) 0 0];
            

%% Rigid Body mass, I, Xtree
for idx_body=1:robot.nd
    robot.jtype{idx_body} = jtype_{idx_body};          % joint type
    robot.com{idx_body}   = com_(idx_body,:);           % Com Position
    robot.mass{idx_body} = mass_(idx_body);                   % Body Mass 
    robot.I{idx_body} = mcI(robot.mass{idx_body}, robot.com{idx_body}, diag(iner_(idx_body,:))); % Inertia
    robot.Xtree{idx_body} = plux(eye(3), jpos_(idx_body,:));   % Tree transform
end

%% outputs
robot.q = params_.q;
robot.qd = params_.qd;
robot.rq = params_.rq;
robot.rqd = params_.rqd;

% % debug
% [H, C] = HandC(robot, robot.rq, robot.rqd);
% robot.H_ = simplify(H);
% 
% syms Irot real
% robot.Hmot_ = robot.H_ - subs(robot.H_, Irot, 0)
% syms a
