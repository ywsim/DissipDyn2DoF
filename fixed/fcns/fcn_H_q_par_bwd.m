function [H] = fcn_H_q_par_bwd(q,L,m,I,gear,eff)

H = zeros(2,2);

  H(1,1)=I(1) + I(2) + gear(2) + (L(1)^2*m(1))/4 + L(1)^2*m(2) + (L(2)^2*m(2))/4 + (gear(1)*...
         gear(3)^2)/eff(1) + (gear(2)*gear(4)^2)/eff(2) + 2*gear(2)*gear(4)*(1/eff(2))^(1/2) + L(1)*L(2)*m(2)*cos(q(4));
  H(1,2)=I(2) + (L(2)^2*m(2))/4 + (gear(2)*gear(4)^2)/eff(2) + gear(2)*gear(4)*(1/eff(2))^(1/2) +...
          (L(1)*L(2)*m(2)*cos(q(4)))/2;
  H(2,1)=I(2) + (L(2)^2*m(2))/4 + (gear(2)*gear(4)^2)/eff(2) + gear(2)*gear(4)*(1/eff(2))^(1/2) +...
          (L(1)*L(2)*m(2)*cos(q(4)))/2;
  H(2,2)=I(2) + (L(2)^2*m(2))/4 + (gear(2)*gear(4)^2)/eff(2);

 