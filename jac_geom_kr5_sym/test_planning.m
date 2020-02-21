q_init=[0;pi/2;0;0;0;0];
v_max=0.5*[375;300;375;410;410;660]*pi/180;
x_d=[0;0.45;1.515];
vc_max=0.8;
ac_max=0.2;

delay=10;

N=500;
dt=0.012;
q=zeros(6,N);
q(:,1)=q_init;
x_init=computeKinematic(q_init);
p=zeros(3,N);
t=zeros(1,N);
sc=zeros(1,N);
qdot=zeros(6,N);
p_des=zeros(3,N);
i_error=zeros(3,1);
v_dest=zeros(3,N);
for k=1:N
    tk=(k-1)*dt;

    L=norm(x_d-x_init);
    [s,sdot]=bang_bang(L,tk,vc_max,ac_max);

    x_des=x_init+s/L*(x_d-x_init);
    v_des=sdot/L*(x_d-x_init);
    v_dest(:,k)=v_des;
    x_e = computeKinematic(q(:,k));
    p_error = x_des - x_e;
    i_error=i_error+dt*p_error;
    v_r=v_des+5*p_error+4*i_error;

    Jacobian=computePosJacobian(q(:,k));

    %cmdVel = dampedPinv(Jacobian)*0.2*p_error;
    %cmdVel = dampedPinv(Jacobian)*v_r;
    %saturate(cmdVel);
    
    cmdVel = pinv(Jacobian)*v_r;
    [cmdVel,sc_k] = scale(cmdVel,v_max);
    qdot(:,k)=cmdVel;
    
    if k<(delay+1) %simulating delay
        qdotk=zeros(6,1);
    else
        qdotk=qdot(:,k-delay);
    end
    q(:,k+1)=q(:,k)+qdotk*dt;
    p(:,k)=x_e;
    t(k)=tk;
    sc(k)=sc_k;
    p_des(:,k)=x_des;
    if mod(k,100)==0
        fprintf('.')
    end
end
fprintf('\n')
q=q(:,1:end-1);

plot3(p(1,:).',p(2,:).',p(3,:).',p_des(1,:).',p_des(2,:).',p_des(3,:).'); grid on; axis on; axis equal;