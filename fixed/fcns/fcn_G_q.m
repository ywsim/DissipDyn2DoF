function [G] = fcn_G_q(gear)

G = zeros(2,2);

  G(1,1)=1/gear(3);
  G(1,2)=0;
  G(2,1)=0;
  G(2,2)=1/gear(4);

 