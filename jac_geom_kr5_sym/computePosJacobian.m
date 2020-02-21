function jac=computePosJacobian(q)

  a1=0.075; a2=0.27; a3=0.09;
  d4=0.295; d6=0.08; %d1=0.132; 
  % h=1.023; %altezza base

  s1=sin(q(1));
  s2=sin(q(2));
  s4=sin(q(4));
  s5=sin(q(5));
  s23=sin(q(2)+q(3));

  c1=cos(q(1));
  c2=cos(q(2));
  c4=cos(q(4));
  c5=cos(q(5));
  c23=cos(q(2)+q(3));
  
  jac=zeros(3,6);

  jac(1,1)=d6*(s5*(-s1*c4*c23+c1*s4)-c5*s1*s23)-d4*s1*s23-a3*s1*c23-a2*s1*c2-a1*s1;
  jac(1,2)=-c1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23+a2*s2);
  jac(1,3)=-c1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23);
  jac(1,4)=d6*s5*(-c1*s4*c23+s1*c4);
  jac(1,5)=d6*(c5*(c1*c4*c23+s1*s4)-s5*c1*s23);
  jac(1,6)=0;

  jac(2,1)=d6*(s5*(c1*c4*c23+s1*s4)+c5*c1*s23)+d4*c1*s23+a3*c1*c23+a2*c1*c2+a1*c1;
  jac(2,2)=-s1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23+a2*s2);
  jac(2,3)=-s1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23);
  jac(2,4)=d6*s5*(-s1*s4*c23-c1*c4);
  jac(2,5)=d6*(c5*(s1*c4*c23-c1*s4)-s5*s1*s23);
  jac(2,6)=0;

  jac(3,1)=0;
  jac(3,2)=d6*(c4*s5*c23+c5*s23)+d4*s23+a3*c23+a2*c2;
  jac(3,3)=d6*(c4*s5*c23+c5*s23)+d4*s23+a3*c23;
  jac(3,4)=d6*(-s4*s5*s23);
  jac(3,5)=d6*(c4*c5*s23+s5*c23);
  jac(3,6)=0;