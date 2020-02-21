q_init=[0;pi/2;0;0;0;0];
v_max=0.5*[375;300;375;410;410;660]*pi/180;
a_max=0.25*v_max;
q_d=[pi/2;pi/2;0;0;0;0];

delay=15;

N=500;
dt=0.012;
q=zeros(6,N);
q(:,1)=q_init;
p=zeros(3,N);
t=zeros(1,N);
sc=zeros(1,N);
qdot=zeros(6,N);
p_des=zeros(3,N);
q_dest=zeros(6,N);
qdot_dest=zeros(6,N);
for k=1:N
    tk=(k-1)*dt;

    L=norm(q_d-q_init);
    [s,sdot]=bang_bang(L,tk,min(v_max),min(a_max));

    q_des=q_init+s/L*(q_d-q_init);
    qdot_des=sdot/L*(q_d-q_init);
    qdot_dest(:,k)=qdot_des;
    q_e = q(:,k);
    q_error = q_des - q_e;

    cmdVel = qdot_des+5*q_error;
    [cmdVel,sc_k] = scale(cmdVel,v_max);
    
    if k<(delay+1) %simulating delay
        qdotk=zeros(6,1);
    else
        qdotk=qdot(:,k-delay);
    end
    q(:,k+1)=q(:,k)+qdotk*dt;
    p(:,k)=computeKinematic(q_e);
    t(k)=tk;
    sc(k)=sc_k;
    qdot(:,k)=cmdVel;
    q_dest(:,k)=q_des;
    p_des(:,k)=computeKinematic(q_des);
    if mod(k,100)==0
        fprintf('.')
    end
end
fprintf('\n')
q=q(:,1:end-1);

plot3(p(1,:).',p(2,:).',p(3,:).',p_des(1,:).',p_des(2,:).',p_des(3,:).'); grid on; axis on; axis equal;