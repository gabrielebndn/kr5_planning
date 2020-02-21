%using euler angles
%#ok<*NOPTS>
q_init=[0;pi/2;0;0;0;0];
%q_init_err=ones(6,1)*pi/180;
END_EFF=0.09;
q_init_err=zeros(6,1);
T_init=computeFullKinematic(q_init);
R_init=T_init(1:3,1:3);
x_init=T_init(1:3,4);
v_max=0.5*[375;300;375;410;410;660]*pi/180;
%x_d=[0;0.45+END_EFF;1.515];
x_d=x_init;
%x_d=[0.45;0;1.515];
% rot_d= [0 -1 0;
%         1  0 0;
%         0  0 1]
rot_d = [0 0 1;
         1 0 0;
         0 1 0];
T_d=[rot_d*R_init,x_d;0 0 0 1];
zyz_d=tr2eul(T_d(1:3,1:3))
zyz_init=tr2eul(T_init(1:3,1:3))
zyz_d=zyz_init+wrapToPi(zyz_d-zyz_init);

vc_max=0.8/2;
ac_max=0.2/2;
va_max=pi/4;
aa_max=pi/8;

K_p=1;
K_i=0.5;
K_o=1;

delay=0;

t_tot=600;
dt=0.012;
N=ceil(t_tot/dt);
q=zeros(6,N);
q(:,1)=q_init+q_init_err;

T=zeros(4,4,N);
t=zeros(1,N);
sc=zeros(1,N);
qdot=zeros(6,N);
T_des=zeros(4,4,N);
v_dest=zeros(3,N);
ang_err_tot=zeros(3,N);
p_err_tot=zeros(3,N);
zyz_e_tot=zeros(3,N);
p_err_tot=zeros(3,N);
i_error=zeros(3,1);
for k=1:N
    tk=(k-1)*dt;
    
    T_e = computeFullKinematic(q(:,k));
    x_e=T_e(1:3,4);
    R_e=T_e(1:3,1:3);
    zyz_e=my_tr2eul(R_e); % zyz_e=tr2eul(R_e);
    
    L=norm(x_d-x_init);
    [Time,Time_s] = bang_bang_init(L,vc_max,ac_max);
    L_zyz=norm(zyz_d-zyz_init);
    [Time_a,Time_as] = bang_bang_init(L_zyz,va_max,aa_max);
    Time_max=max(Time,Time_a);
    frac=Time/Time_max;
    frac_a=Time_a/Time_max;
    [s,sdot]=bang_bang_calc(Time,Time_s,tk*frac,ac_max);
    sdot=sdot*frac;
    [st,stdot]=bang_bang_calc(Time_a,Time_as,tk*frac_a,aa_max);
    stdot=stdot*frac_a;

    if L>0
        x_des=x_init+s/L*(x_d-x_init);
        v_des=sdot/L*(x_d-x_init);
    else
        x_des=x_d;
        v_des=zeros(3,1);
    end
    p_error = x_des - x_e;
    i_error=i_error+dt*p_error;
    v_r=v_des+K_p*p_error+K_i*i_error;
    
    
    if L_zyz>0
        zyz_des=zyz_init+st/L_zyz*(zyz_d-zyz_init);
        angv_des=stdot/L_zyz*(zyz_d-zyz_init);
    else
        zyz_des=zyz_d;
        angv_des=zeros(3,1);
    end
    ang_err = (zyz_des-zyz_e).';
    angv_r=angv_des.'+K_o*wrapToPi(ang_err);
    vo_r=[v_r;angv_r];

    Jacobian=computeFullJacobian(q(:,k));
    Jacobian(4:6,:)=dls_pinv(my_eul2jac(zyz_e))*Jacobian(4:6,:); % Jacobian(4:6,:)=eul2jac(zyz_e)\Jacobian(4:6,:);
    
    cmdVel = dls_pinv(Jacobian)*vo_r; % cmdVel = pinv(Jacobian)*vo_r;
    [cmdVel,sc_k] = scale(cmdVel,v_max);
    qdot(:,k)=cmdVel;
    
    if k<(delay+1) %simulating delay
        qdotk=zeros(6,1);
    else
        qdotk=qdot(:,k-delay);
    end
    q(:,k+1)=q(:,k)+qdotk*dt;
    T(:,:,k)=T_e;
    t(k)=tk;
    ang_err_tot(:,k)=ang_err;
    zyz_e_tot(:,k)=zyz_e.';
    p_err_tot(:,k)=p_error;
    %sc(k)=sc_k;
    if mod(k,100)==0
        fprintf('.')
    end
end
zyz_e=zyz_e_tot(:,1).'
fprintf('\n')
q=q(:,1:end-1);
p=permute(T(1:3,4,:),[1 3 2]);

figure;
plot(zyz_e_tot.'); title zyz\_e;

figure;
plot(ang_err_tot.'); title ang\_err;

figure;
plot3(p(1,:).',p(2,:).',p(3,:).'); grid on; axis on; axis equal;

%dummy=figure;
%set(dummy,'Visible','off');
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
    
    d_6=d_6+END_EFF;

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