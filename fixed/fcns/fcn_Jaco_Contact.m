function [jaco_contact] = fcn_Jaco_Contact(q,L)

jaco_contact = zeros(2,5);

  jaco_contact(1,1)=1;
  jaco_contact(1,2)=0;
  jaco_contact(1,3)=- L(2)*sin(q(3) + q(4)) - L(3)*sin(q(3) + q(4) + q(5));
  jaco_contact(1,4)=- L(2)*sin(q(3) + q(4)) - L(3)*sin(q(3) + q(4) + q(5));
  jaco_contact(1,5)=-L(3)*sin(q(3) + q(4) + q(5));
  jaco_contact(2,1)=0;
  jaco_contact(2,2)=1;
  jaco_contact(2,3)=L(2)*cos(q(3) + q(4)) + L(3)*cos(q(3) + q(4) + q(5));
  jaco_contact(2,4)=L(2)*cos(q(3) + q(4)) + L(3)*cos(q(3) + q(4) + q(5));
  jaco_contact(2,5)=L(3)*cos(q(3) + q(4) + q(5));

 