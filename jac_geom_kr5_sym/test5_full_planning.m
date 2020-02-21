q_init=[0;pi/2;0;0;0;0];
%q_init_err=ones(6,1)*pi/180;
q_init_err=zeros(6,1);
T_init=computeFullKinematic(q_init);
x_init=T_init(1:3,4);
R_init=T_init(1:3,1:3);
v_max=0.5*[375;300;375;410;410;660]*pi/180;
x_d=[0;0.45;1.515];
% rot_d= [0 -1 0;
%         1  0 0;
%         0  0 1]
rot_d = [0 0 1;
         1 0 0;
         0 1 0];
T_d=[rot_d*R_init,x_d;0 0 0 1];

vc_max=0.8/2;
ac_max=0.2/2;
va_max=pi/4;
aa_max=pi/8;

K_p=2;
K_i=1;
K_o=2;

delay=10;

t_tot=10;
dt=0.012;
N=ceil(t_tot/dt);
q=zeros(6,N);
q(:,1)=q_init+q_init_err;

L=norm(x_d-x_init);
[Time,Time_s]    = bang_bang_init(L,vc_max,ac_max);
[th,r]=tr2angvec(T_init(1:3,1:3).'*T_d(1:3,1:3));
r=r.';
[Time_a,Time_as] = bang_bang_init(abs(th),va_max,aa_max);
Time_max=max(Time,Time_a);
time_scale=Time/Time_max;
time_scale_a=Time_a/Time_max;

T=zeros(4,4,N);
t=zeros(1,N);
sc=zeros(1,N);
qdot=zeros(6,N);
T_des=zeros(4,4,N);
i_error=zeros(3,1);
v_dest=zeros(3,N);
for k=1:N
    tk=(k-1)*dt;

    
    [s,sdot]=bang_bang_calc(Time,Time_s,tk*time_scale,ac_max);
    sdot=sdot*time_scale;

    x_des=x_init+s/L*(x_d-x_init);
    v_des=sdot/L*(x_d-x_init);
    v_dest(:,k)=v_des;
    T_e = computeFullKinematic(q(:,k));
    x_e=T_e(1:3,4);
    p_error = x_des - x_e;
    i_error=i_error+dt*p_error;
    v_r=v_des+K_p*p_error+K_i*i_error;
    
    
    [st,stdot]=bang_bang_calc(Time_a,Time_as,tk*time_scale_a,aa_max);
    stdot=stdot*time_scale_a;
    th_des=st*sign(th);
    thdot_des=sign(th)*stdot;
    
    R_des=R_init*angvec2r(th_des,r);
    R_e=T_e(1:3,1:3);
    omg_des=R_init*thdot_des*r;
    [a_err,r_err]=tr2angvec(R_des*R_e.');
    r_err=r_err.';
    ang_err=sin(a_err)*r_err;
    matL=getL(R_des,R_e);
    omg_r=matL\(matL.'*omg_des+K_o*ang_err);

    Jacobian=computeFullJacobian(q(:,k));
    
    %cmdVel = pinv(Jacobian)*[v_r;omg_r];
    cmdVel = dls_pinv(Jacobian)*[v_r;omg_r];
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
    %sc(k)=sc_k;
    T_des(:,:,k)=[R_des,x_des;0 0 0 1];
    if mod(k,100)==0
        fprintf('.')
    end
end
fprintf('\n')
q=q(:,1:end-1);
p=permute(T(1:3,4,:),[1 3 2]);
p_des=permute(T_des(1:3,4,:),[1 3 2]);
pos_err=sqrt(sum((p_des-p).^2,1));

plot3(p(1,:).',p(2,:).',p(3,:).',p_des(1,:).',p_des(2,:).',p_des(3,:).'); grid on; axis on; axis equal;

%dummy=figure;
%set(dummy,'Visible','off');
disp('plotting actual orientation');
figure;
plot_triangle(T,'actual orientation',0.012);
disp('actual orientation over');
disp('plotting desired orientation');
figure;
plot_triangle(T_des,'desired orientation',0.012);
disp('desired orientation over');

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