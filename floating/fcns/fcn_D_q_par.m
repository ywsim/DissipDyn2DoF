function [D] = fcn_D_q_par()

D = zeros(5,5);

  D(1,1)=1;
  D(1,2)=0;
  D(1,3)=0;
  D(1,4)=0;
  D(1,5)=0;
  D(2,1)=0;
  D(2,2)=1;
  D(2,3)=0;
  D(2,4)=0;
  D(2,5)=0;
  D(3,1)=0;
  D(3,2)=0;
  D(3,3)=1;
  D(3,4)=0;
  D(3,5)=0;
  D(4,1)=0;
  D(4,2)=0;
  D(4,3)=0;
  D(4,4)=1;
  D(4,5)=0;
  D(5,1)=0;
  D(5,2)=0;
  D(5,3)=0;
  D(5,4)=-1;
  D(5,5)=1;

 