function robot = get_dynamics(robot, flagWriteFcn)
%% import params
params_ = import_robot_params('symbolic');
q = params_.q;
rq = params_.rq;
qL = q(robot.nBase+1:end);
p = params_.p;
n_bwd = params_.n_bwd;
n_fwd = params_.n_fwd;
% gear = params_.gear;
% eff = params_.eff;

% import transmission
G = robot.tran.G;
Dser = robot.tran.D.ser;
Dpar = robot.tran.D.par;
Ddif = robot.tran.D.dif;

%% Calc H and C
[H_raw, C_raw] = HandC(robot, robot.rq, robot.rqd);

% H and C in redundant q
H_rq = simplify(H_raw);
C_rq = simplify(C_raw);


%% Shrink Model with Constraints
% Constraint w.r.t. redundant q 
Cser = p - inv(G)*(Dser\qL);
Cpar = p - inv(G)*(Dpar\qL);
Cdif = p - inv(G)*(Ddif\qL);

% Constraint Jacobian
Aser = jacobian(Cser,rq);
Apar = jacobian(Cpar,rq);
Adif = jacobian(Cdif,rq);

inv_G_q = blkdiag(eye(robot.nBase), inv(G));
inv_Dser_q = blkdiag(eye(robot.nBase), inv(Dser));
inv_Dpar_q = blkdiag(eye(robot.nBase), inv(Dpar));
inv_Ddif_q = blkdiag(eye(robot.nBase), inv(Ddif));

% Constraint Null Matrix 
Kser = null(Aser)*inv_G_q*inv_Dser_q;
Kpar = null(Apar)*inv_G_q*inv_Dpar_q;
Kdif = null(Adif)*inv_G_q*inv_Ddif_q;

% Efficiency Matrix 
Ebwd = diag([ones(robot.nBase,1);  ones(robot.nLimbs,1);   1./n_bwd]);
Efwd = diag([ones(robot.nBase,1);  ones(robot.nLimbs,1);   n_fwd]);
Ebwd_q = diag([ones(robot.nBase,1);  1./n_bwd]);
Efwd_q = diag([ones(robot.nBase,1);  n_fwd]);

% Reduced System's Inertia Matrix
reduced.H_q.ser.bwd = simplify(Kser'*sqrt(Ebwd)*H_rq*sqrt(Ebwd)*Kser);      % Serial, Reduced, back
reduced.H_q.ser.fwd = simplify(Kser'*sqrt(Efwd)*H_rq*sqrt(Efwd)*Kser);      % Serial, Reduced, forw
reduced.H_q.ser.idl = simplify(Kser'*H_rq*Kser);                            % Serial, Reduced, Ideal

reduced.H_q.par.bwd = simplify(Kpar'*sqrt(Ebwd)*H_rq*sqrt(Ebwd)*Kpar);
reduced.H_q.par.fwd = simplify(Kpar'*sqrt(Efwd)*H_rq*sqrt(Efwd)*Kpar);
reduced.H_q.par.idl = simplify(Kpar'*H_rq*Kpar);

reduced.H_q.dif.bwd = simplify(Kdif'*sqrt(Ebwd)*H_rq*sqrt(Ebwd)*Kdif);
reduced.H_q.dif.fwd = simplify(Kdif'*sqrt(Efwd)*H_rq*sqrt(Efwd)*Kdif);
reduced.H_q.dif.idl = simplify(Kdif'*H_rq*Kdif);

reduced.K.ser = Kser;
reduced.K.par = Kpar;
reduced.K.dif = Kdif;

%% outputs
robot.reduced = reduced;
robot.H_rq = H_rq;
robot.C_rq = C_rq;

% write function
v_list = import_v_list();
passing_var = {'q','L','m','I','gear','eff'};
v_list_var = [v_list.q; v_list.L; v_list.m; v_list.I; v_list.gear; v_list.eff];
if flagWriteFcn
    cd fcns
    write_fcn_m('fcn_G_q.m', {'gear'}, v_list.gear, {inv(inv_G_q), 'G'});
    write_fcn_m('fcn_D_q_ser.m', {}, [], {inv(inv_Dser_q), 'D'});
    write_fcn_m('fcn_D_q_par.m', {}, [], {inv(inv_Dpar_q), 'D'});
    write_fcn_m('fcn_D_q_dif.m', {}, [], {inv(inv_Ddif_q), 'D'});
    write_fcn_m('fcn_E_rq_bwd.m', {'eff'}, v_list.eff, {Ebwd, 'E'});
    write_fcn_m('fcn_E_rq_fwd.m', {'eff'}, v_list.eff, {Efwd, 'E'});
    write_fcn_m('fcn_E_q_bwd.m', {'eff'}, v_list.eff, {Ebwd_q, 'E'});
    write_fcn_m('fcn_E_q_fwd.m', {'eff'}, v_list.eff, {Efwd_q, 'E'});
    write_fcn_m('fcn_H_q_ser_bwd.m', passing_var, v_list_var, {reduced.H_q.ser.bwd, 'H'});
    write_fcn_m('fcn_H_q_par_bwd.m', passing_var, v_list_var, {reduced.H_q.par.bwd, 'H'});
    write_fcn_m('fcn_H_q_dif_bwd.m', passing_var, v_list_var, {reduced.H_q.dif.bwd, 'H'});
    write_fcn_m('fcn_H_q_ser_fwd.m', passing_var, v_list_var, {reduced.H_q.ser.fwd, 'H'});
    write_fcn_m('fcn_H_q_par_fwd.m', passing_var, v_list_var, {reduced.H_q.par.fwd, 'H'});
    write_fcn_m('fcn_H_q_dif_fwd.m', passing_var, v_list_var, {reduced.H_q.dif.fwd, 'H'});
    write_fcn_m('fcn_H_q_ser_idl.m', passing_var, v_list_var, {reduced.H_q.ser.idl, 'H'});
    write_fcn_m('fcn_H_q_par_idl.m', passing_var, v_list_var, {reduced.H_q.par.idl, 'H'});
    write_fcn_m('fcn_H_q_dif_idl.m', passing_var, v_list_var, {reduced.H_q.dif.idl, 'H'});
    cd ..
    disp('- writing completed')
end