function q=plan_lin_eul(robot,p_init,p_end,q_init,q_end,t,vcart_max,acart_max)

t_l=length(t);
dt=t(2)-t(1);
n_j=length(q_init);
q=zeros(n_j,t_l);
q(:,1)=q_init;

%pos_tol=0.1;
%eul_tol=0.1;

pos_end = p_end(1:3);
eul_end = p_end(4:6);

for k=2:t_l
    T = robot.fkine(q(:,k-1).');
    pos = T(1:3,4);
    eul = tr2eul(T).';

    pos_e = pos_end-pos;
    pos_en = norm(pos_e);
    pos_e = pos_e/pos_en;
    eul_e = eul_end-eul;
    eul_en = norm(eul);
    eul_e = eul_e/eul_en;
    
    v_pos = pos_e;
    v_eul = eul_e;
    
    v = [v_pos;v_eul];
    
    jac = robot.jacob0(q(:,k-1).','eul');
    
    qdot = pinv(jac)*v;
    
    q(:,k)=q(:,k-1)+qdot*dt;
end
    