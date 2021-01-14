% function robot = gen_floating_leg()
%% Init
addpath(genpath(pwd))
clear;clc;

robot.name = '2 DOF 2-Link Planar Monoped';
robot.base = 'floating';
robot.flagFloat = false;
robot.drive = 'serial';
robot.dimensions = 'planar';
robot.nd = 5;           % number of DoF
robot.NB = robot.nd;    % number of body
robot.parent = [0 1 2 3 4];  

%% Import Params
params_ = import_robot_params(robot, 'symbolic');

%% Rigid Body Params

mass_ = [0; 0; params_.m];

com_ = [    zeros(3,3);
            params_.L(2)/2 0 0;
            params_.L(3)/2 0 0];

iner_ = [   zeros(2,3);
            0 0 params_.I(1);
            0 0 params_.I(2);
            0 0 params_.I(3)];
     
jtype_ = {'Px', 'Py', 'Rz', 'Rz','Rz'};

jpos_ = [   zeros(4,3)
            params_.L(2) 0 0];
            
%% Contact Params 
cpos_ = [params_.L(3) 0 0];

%% Rigid Body mass, I, Xtree
for idx_body=1:robot.nd
    robot.jtype{idx_body} = jtype_{idx_body};          % joint type
    robot.com{idx_body}   = com_(idx_body,:);           % Com Position
    robot.mass{idx_body} = mass_(idx_body);                   % Body Mass 
    robot.I{idx_body} = mcI(robot.mass{idx_body}, robot.com{idx_body}, diag(iner_(idx_body,:))); % Inertia
    robot.Xtree{idx_body} = plux(eye(3), jpos_(idx_body,:));   % Tree transform
end

q = params_.q;
qd = params_.qd;

jaco_xyz = contactJaco(robot, q, qd, robot.flagFloat , robot.nd, cpos_);
jaco_contact = simplify(jaco_xyz(1:2, :));


%% HandC
[Hc, Cc] = HandC(robot, q, qd);
H = simplify(Hc);
C = simplify(Cc);
robot.H = H;
robot.C = C;
%%
v_list = import_v_list();
%%

cd fcns
write_fcn_m('fcn_H.m', {'q','L','m','I'}, [v_list.q; v_list.L; v_list.m; v_list.I], {H, 'H'});
write_fcn_m('fcn_C.m', {'q','qd','L','m','I'}, [v_list.q; v_list.qd; v_list.L; v_list.m; v_list.I], {C, 'C'});
write_fcn_m('fcn_Jaco_Contact.m', {'q','L'}, [v_list.q; v_list.L], {jaco_contact, 'jaco_contact'});
cd ..
disp('- writing completed')

%% outputs
% robot contact points
% robot.contact.rWF = model_prms.rWF;
% robot.contact.rWB = model_prms.rWB;
  
% robot.jpos = jpos_;
robot.q = q ;
robot.qd = qd;
robot.jaco_contact = jaco_contact;

%%
cd dynamics
save('robot_dynamics.mat', 'robot');
cd ..
disp('- saved robot dynamics')
