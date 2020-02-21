%#ok<*NOPTS>
clear all;
q_init=[0;pi/2;0;0;0;0];
T_init=computeFullKinematic(q_init);
R_init=T_init(1:3,1:3);
v_max=0.5*[375;300;375;410;410;660]*pi/180;
x_d=[0;0.45;1.515];
rot_d = [0 0 1;
         1 0 0;
         0 1 0];
% rot_d= [0 -1 0;
%         1  0 0;
%         0  0 1]
T_d=[rot_d*R_init,x_d;0 0 0 1];
zyz_init=my_tr2eul(T_init(1:3,1:3)).'
zyz_d=my_tr2eul(T_d(1:3,1:3)).'
%zyz_d=zyz_init+((rand(3,1)<0.5)*2-1)*2*pi/180;

v_max=0.5*[375;300;375;410;410;660]*pi/180;

K_p=1;
K_o=1;

delay=10;

t_tot=10;
dt=0.012;
N=ceil(t_tot/dt);
q=zeros(6,N);
q(:,1)=q_init;

T=zeros(4,4,N);
t=zeros(1,N);
qdot=zeros(6,N);
T_des=zeros(4,4,N);
zyz_err_tot=zeros(3,N);
zyz_e_tot=zeros(3,N);
zyz_des=zeros(3,N);
p=zeros(3,N);
vo_r_tot=zeros(6,N);
%vo_r_tot=zeros(3,N);

for k=1:N
    q_e=q(:,k);
    T_e=computeFullKinematic(q_e);
    R_e=T_e(1:3,1:3);
    x_e=T_e(1:3,4);
    zyz_e=my_tr2eul(R_e).';
    
    x_err=x_d-x_e;
    v_r=K_p*x_err;
    zyz_err=zyz_d-zyz_e;
    angv_r=K_o*wrapToPi(zyz_err);
    vo_r=[v_r;angv_r];
    %vo_r=angv_r;
    
    jac=computeFullJacobian(q_e);
    convertM=my_eul2jac(zyz_e);
    jac(4:6,:)=pinv(convertM)*jac(4:6,:);
    
    cmdVel=pinv(jac)*vo_r;
    cmdVel = scale(cmdVel,v_max);
    
    qdot(:,k)=cmdVel;
    
    if k<(delay+1) %simulating delay
        qdotk=zeros(6,1);
    else
        qdotk=qdot(:,k-delay);
    end
    
    
    q(:,k+1)=q_e+qdotk*dt;
    
    T(:,:,k)=T_e;
    p(:,k)=x_e;
    t(k)=(k-1)*dt;
    qdot(:,k)=cmdVel;
    T_des(:,:,k)=T_d;
    zyz_err_tot(:,k)=zyz_err;
    zyz_e_tot(:,k)=zyz_e;
    zyz_des(:,k)=zyz_d;
    vo_r_tot(:,k)=vo_r;
end
q=q(:,1:end-1);



figure;
plot(zyz_e_tot.'); title zyz\_e;

figure;
plot(zyz_des.'); title zyz\_des;

figure;
plot(zyz_err_tot.'); title ang\_err;

figure;
plot(wrapToPi(zyz_err_tot).'); title ang\_err\_toPi;

figure;
plot(abs(wrapToPi(zyz_err_tot)).'); title abs\_ang\_err\_toPi;

figure;
plot3(p(1,:).',p(2,:).',p(3,:).'); grid on; axis on; axis equal;



% zyz_totsum=zyz_e_tot+vo_r_tot*dt;
% zyz_diff=[zyz_e_tot(:,1), zyz_totsum(:,1:end-1)]-zyz_e_tot;
% figure;
% plot(zyz_diff.'); title hallo;
% figure;
% plot(wrapToPi(zyz_diff).'); title hallo\_toPi;



disp('plotting actual orientation');
figure;
plot_triangle(T,'actual orientation',0.012);
disp('actual orientation over');

try
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
    kr5.plot(q.','delay',0.012)
catch %#ok<CTCH>
    disp('!! Plot error before the end !!');
end
disp('plotting over');
