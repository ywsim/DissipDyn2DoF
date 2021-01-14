function [E] = fcn_E_q_fwd(eff)

E = zeros(2,2);

  E(1,1)=eff(3);
  E(1,2)=0;
  E(2,1)=0;
  E(2,2)=eff(4);

 