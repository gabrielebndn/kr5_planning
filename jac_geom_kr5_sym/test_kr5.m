d_1=0.132;
d_4=0.295;
d_6=0.080;

a_1=0.075;
a_2=0.270;
a_3=0.090;

kr5=SerialLink(... % theta d a alpha
    [0	d_1	a_1   pi/2;
     0  0	a_2      0;
     0  0   a_3   pi/2;
     0	d_4	0    -pi/2;
     0	0	0     pi/2;
     0	d_6	0        0],...
     'name','kuka\_kr5');
 
kr5.plot(zeros(1,6)+[0 pi/2 0 0 0 0]+0.35*ones(1,6));
%kr5.plot(zeros(1,6)+[0 pi/2 0 pi/4 0 0]);