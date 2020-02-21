%#ok<*NOPTS>
q_init=[0;pi/2;0;0;0;0];

%q=[pi/2;pi/2;0;0;0;0];
q=[pi/2;pi/2;0;0;0;pi/2];

d_1=0.132;
d_4=0.295;
d_6=0.080;

a_1=0.075;
a_2=0.270;
a_3=0.090;

h=1.023;
d_1=d_1+h;

kr5=SerialLink(... % theta d a alpha
    [0	d_1	a_1   pi/2;
     0  0	a_2      0;
     0  0   a_3   pi/2;
     0	d_4	0    -pi/2;
     0	0	0     pi/2;
     0	d_6	0        0],...
     'name','kuka\_kr5');
 
figure;
kr5.plot(q_init.');
title init

figure;
kr5.plot(q.')
title des

kin_r=kr5.fkine(q);
R_r=kin_r(1:3,1:3)

kin_init=kr5.fkine(q_init);
R_init=kin_init(1:3,1:3);
% rot= [0 -1 0;
%       1  0 0;
%       0  0 1]
rot = [0 0 1;
       1 0 0;
       0 1 0];
R_m=rot*R_init

      



 
