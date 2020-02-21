function T=computeFullKinematic(q)

  a1=0.075; a2=0.27; a3=0.09;
  d1=0.132; d4=0.295; d6=0.08;
  h=1.023; %altezza base
  END_EFF=0.09;
  d6=d6+END_EFF;
  
  s1=sin(q(1));
  s2=sin(q(2));
  s4=sin(q(4));
  s5=sin(q(5));
  s23=sin(q(2)+q(3));
  s6=sin(q(6));

  c1=cos(q(1));
  c2=cos(q(2));
  c4=cos(q(4));
  c5=cos(q(5));
  c23=cos(q(2)+q(3));
  c6=cos(q(6));
  
  pos=zeros(3,1);

  pos(1)=d6*(s5*(c1*c4*c23+s1*s4)+c5*c1*s23)+d4*c1*s23+a3*c1*c23+a2*c1*c2+a1*c1;
  pos(2)=d6*(s5*(s1*c4*c23-c1*s4)+c5*s1*s23)+d4*s1*s23+a3*s1*c23+a2*s1*c2+a1*s1;
  pos(3)=h+d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23+a2*s2+d1;

  rot=zeros(3);

  rot(1,1) = s6*(s1*c4-c23*c1*s4)+c6*(c5*(s1*s4+c23*c1*c4)-s23*c1*s5);
  rot(1,2) = c6*(s1*c4-c23*c1*s4)-s6*(c5*(s1*s4+c23*c1*c4)-s23*c1*s5);
  rot(1,3) = s5*(s1*s4+c23*c1*c4)+s23*c1*c5;

  rot(2,1) =-s6*(c1*c4+c23*s1*s4)-c6*(c5*(c1*s4-c23*s1*c4)+s23*s1*s5);
  rot(2,2) =-c6*(c1*c4+c23*s1*s4)+s6*(c5*(c1*s4-c23*s1*c4)+s23*s1*s5);
  rot(2,3) =-s5*(c1*s4-c23*s1*c4)+s23*s1*c5;

  rot(3,1) = c6*(c23*s5+s23*c4*c5)-s23*s4*s6;
  rot(3,2) =-s6*(c23*s5+s23*c4*c5)-s23*s4*c6;
  rot(3,3) = s23*c4*s5-c23*c5;

  T=[rot,pos;0 0 0 1];