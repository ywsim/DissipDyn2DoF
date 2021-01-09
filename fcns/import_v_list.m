function v_list = import_v_list()
%% Refer to import_robot_params
% syms qx qy real
% syms qdx qdy real
% syms L I m q qd [3 1 ] real
% syms p pd n_bwd n_fwd [2 1] real
% syms Irot N real

%% List
v_list.q = {
    'qx' 'q(1)';
    'qy' 'q(2)';
    'q1' 'q(3)';
    'q2' 'q(4)';
    'q3' 'q(5)';
    };

v_list.qd = {
    'qdx' 'qd(1)';
    'qdy' 'qd(2)';
    'qd1' 'qd(3)';
    'qd2' 'qd(4)';
    'qd3' 'qd(5)';
    };

v_list.L = {
    'L1' 'L(1)';
    'L2' 'L(2)';
    'L3' 'L(3)';
    };

v_list.m = {
    'm1' 'm(1)';
    'm2' 'm(2)';
    'm3' 'm(3)';
    };

v_list.I = {
    'I1' 'I(1)';
    'I2' 'I(2)';
    'I3' 'I(3)';
    };

v_list.gear = {
    'Irot1'  'gear(1)';
    'Irot2'  'gear(2)';
    'N1'     'gear(3)';
    'N2'     'gear(4)';
    };

v_list.eff = {
    'n_bwd1' 'eff(1)';
    'n_bwd2' 'eff(2)';
    'n_fwd1' 'eff(3)';
    'n_fwd2' 'eff(4)';
    };

end
