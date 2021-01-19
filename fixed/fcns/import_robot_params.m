function out = import_robot_params(flag_symbolic_value)
% flag_symbolic_value should be either 'symbolic' or 'value'

switch flag_symbolic_value
    case 'symbolic'
        syms L I m q qd [2 1 ] real
        syms p pd n_bwd n_fwd [2 1] real
        syms Irot N [2 1] real
        
        out.L = L;
        out.m = m;
        out.I = I;
        
        out.q =  q;
        out.qd = qd;
        out.rq = [q;p];
        out.rqd= [qd;pd];
        
        out.p =  p;
        out.pd = pd;
        
        out.N = N;
        out.Irot = Irot;
        out.gear = [Irot;N];
        out.n_bwd = [n_bwd1;n_bwd2];
        out.n_fwd = [n_fwd1;n_fwd2];
        out.eff = [n_bwd;n_fwd];
        
    case 'value'
        out.L = [0.4; 0.4];
        out.m = [2; 2];
        out.I = [1/2; 1/12] .* out.L.^2 .* out.m;
   
        
    otherwise 
        disp('Unknown method.')
end