function [jaco_contact] = fcn_Jaco_Contact(q,L)

jaco_contact = zeros(2,2);

  jaco_contact(1,1)=- L(2)*sin(q(1) + q(2)) - L(1)*sin(q(1));
  jaco_contact(1,2)=-L(2)*sin(q(1) + q(2));
  jaco_contact(2,1)=L(2)*cos(q(1) + q(2)) + L(1)*cos(q(1));
  jaco_contact(2,2)=L(2)*cos(q(1) + q(2));

 