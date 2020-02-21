%#ok<*NOPTS>
%q=[0;pi/2;0;0;0;0]
%q=[0.3;-0.1;pi/4;0.2;0.1;-0.2]
%q=[0;0;0;0;0;0]
%q=pi/2*ones(6,1)
q=pi*ones(6,1)

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
 
kin_r=kr5.fkine(q)
kin_m=computeFullKinematic(q)

jac_r=kr5.jacob0(q)
jac_m=computeFullJacobian(q)

kin_diff=max(max(abs(kin_r-kin_m)))
jac_diff=max(max(abs(jac_r-jac_m)))
 
