function kine=showcomp_kine(q)
    disp('note: using default DH:');
    a1=0.075; a2=0.27; a3=0.09;
    d1=0.132; d4=0.295; d6=0.08;
    END_EFF=0.09;
    d6=d6+END_EFF;
    dh=[ 0	d1	a1   pi/2;
         0  0	a2      0;
         0  0   a3   pi/2;
         0	d4	0   -pi/2;
         0	0	0    pi/2;
         0	d6	0       0] %#ok<NOPRT>
     
     kuka_robot=SerialLink(dh);
     
     figure;
     kuka_robot.plot(q.');
     kine=computeFullKinematic(q);
     