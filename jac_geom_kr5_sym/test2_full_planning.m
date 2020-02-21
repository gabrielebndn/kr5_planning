%computes desired config with inv_kin & inputs appropriate vel w/ control
q_init=[0;pi/2;0;0;0;0];
%q_init_err=ones(6,1)*pi/180;
q_init_err=zeros(6,1);
T_init=computeFullKinematic(q_init);
x_init=T_init(1:3,4);
R_init=T_init(1:3,1:3);
v_max=0.5*[375;300;375;410;410;660]*pi/180;
x_d=[0;0.45;1.515];
T_d=[[0 -1 0;1 0 0; 0 0 1]*R_init,x_d;0 0 0 1];
vc_max=0.8/10;
ac_max=0.2/10;
va_max=pi/16;
aa_max=pi/32;

delay=10;

t_tot=10;
dt=0.012;
N=ceil(t_tot/dt);
q=zeros(6,N);
q(:,1)=q_init+q_init_err;

T=zeros(4,4,N);
t=zeros(1,N);
sc=zeros(1,N);
qdot=zeros(6,N);
T_des=zeros(4,4,N);
i_error=zeros(3,1);
v_dest=zeros(3,N);
for k=1:N
    tk=(k-1)*dt;
    q_e=q(:,k);
    T_e = computeFullKinematic(q_e);

    L=norm(x_d-x_init);
    [s,sdot]=bang_bang(L,tk,vc_max,ac_max);
    x_des=x_init+s/L*(x_d-x_init);
    
    [th,r]=tr2angvec(T_init(1:3,1:3).'*T_d(1:3,1:3)); r=r.';
    [st,stdot]=bang_bang(abs(th),tk,va_max,aa_max);
    th_des=st*sign(th);   
    R_des=R_init*angvec2r(th_des,r);
    
    Tk_des=[R_des,x_des;0 0 0 1];
    
    q_des=inv_kin(Tk_des,q_e);
    
    cmdVel = (q_des-q_e)/dt;
    
    [cmdVel,sc_k] = scale(cmdVel,v_max);
    qdot(:,k)=cmdVel;
    
    if k<(delay+1) %simulating delay
        qdotk=zeros(6,1);
    else
        qdotk=qdot(:,k-delay);
    end
    q(:,k+1)=q_e+qdotk*dt;
    T(:,:,k)=T_e;
    t(k)=tk;
    sc(k)=sc_k;
    T_des(:,:,k)=[R_des,x_des;0 0 0 1];
    if mod(k,100)==0
        fprintf('.')
    end
end
fprintf('\n')
q=q(:,1:end-1);
p=permute(T(1:3,4,:),[1 3 2]);
p_des=permute(T_des(1:3,4,:),[1 3 2]);

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
catch err
    disp('!! Plot error before the end !!');
end
disp('plotting over');