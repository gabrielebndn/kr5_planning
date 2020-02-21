function [q,s]=plan_lin_axang(robot,T_init,T_end,q_init,t,vcart_max,acart_max,qdot_max)

dt = t(2)-t(1);

pos_init=T_init(1:3,4);
pos_end=T_end(1:3,4);
R_init=T_init(1:3,1:3);
R_end=T_end(1:3,1:3);

[Dth,r]=tr2angvec(R_init.'*R_end);
s_max = [norm(pos_end-pos_init);Dth];

s=bang_bang_sync(s_max,t,vcart_max,acart_max);

t_l=length(t);
n_j=length(q_init);
q=zeros(n_j,t_l);
q(:,1)=q_init;

% % WRONG ORIENTATION?
% T_old = robot.fkine(q_init);
% for k=2:t_l
%     pos_new = pos_init + s(1,k)/L(1)*(pos_end-pos_init);
%     R_new = R_init*angvec2r(s(2,k),r);
%     
%     v = (pos_new-T_old(1:3,4))/dt;
%     [omg_n,omg_v] = tr2angvec(T_old(1:3,1:3).'*R_new);
%     omg = omg_n*dt*(omg_v.');
%     
%     jac = robot.jacob0(q(:,k-1));
%     qdot = pinv(jac)*[v;omg];
%     qdot = scale_v(qdot,qdot_max);
%     
%     q(:,k)=q(:,k-1)+qdot*dt;
%     
%     T_old = robot.fkine(q(:,k));
% end

% % works!!
% T_old = robot.fkine(q_init);
% for k=2:t_l
%     pos_new = pos_init + s(1,k)/L(1)*(pos_end-pos_init);
%     R_new = R_init*angvec2r(s(2,k),r);
%     
%     v = (pos_new-T_old(1:3,4))/dt;
%     [omg_n,omg_v] = tr2angvec(R_new*T_old(1:3,1:3).');
%     omg = omg_n/dt*(omg_v.');
%     
%     jac = robot.jacob0(q(:,k-1));
%     qdot = pinv(jac)*[v;omg];
%     qdot = scale_v(qdot,qdot_max);
%     
%     q(:,k)=q(:,k-1)+qdot*dt;
%     
%     T_old = robot.fkine(q(:,k));
% end

% % ~BOOK?
% T_old = robot.fkine(q_init);
% v_ver=(pos_end-pos_init)/norm(pos_end-pos_init);
% K=eye(6);
% for k=2:t_l
%     pos_new = pos_init + s(1,k)/L(1)*(pos_end-pos_init);
%     R_new = R_init*angvec2r(s(2,k),r);
%     
%     v_d = (s(1,k)-s(1,k-1))/dt*v_ver;
%     omg_d = R_init*((s(2,k)-s(2,k-1))/dt*(r.'));
%     
%     pos_err = (pos_new-T_old(1:3,4))/dt;
%     [angle_err,axis_err] = tr2angvec(R_new*T_old(1:3,1:3).');
%     ang_err = angle_err/dt*(axis_err.');
%     
%     jac = robot.jacob0(q(:,k-1));
%     qdot = pinv(jac)*([v_d;omg_d]+K*[pos_err;ang_err]);
%     qdot = scale_v(qdot,qdot_max);
%     
%     q(:,k)=q(:,k-1)+qdot*dt;
%     
%     T_old = robot.fkine(q(:,k));
% end
% disp('hallo')

% AS BOOK
v_ver=(pos_end-pos_init)/norm(pos_end-pos_init);
K_p=100*eye(3);
K_o=100*eye(3);
t_l1=t_l-1;
for k=1:t_l1
    if t(k)==1.28
        disp('stop. cont');
    end
    pos_des = pos_init + s(1,k)/s_max(1)*(pos_end-pos_init);
    R_des = R_init*angvec2r(s(2,k),r);
    T=robot.fkine(q(:,k));
    R=T(1:3,1:3);
    pos=T(1:3,4);
    
    v_d = (s(1,k+1)-s(1,k))/dt*v_ver;
    omg_d = R_init*((s(2,k+1)-s(2,k))/dt*(r.'));
    
    pos_err = pos_des-pos;
    [angle_err,axis_err] = tr2angvec(R_des*R.');
    ang_err = sin(angle_err)*(axis_err.');
    
    v_r=v_d+K_p*pos_err;   
    L=-0.5*(skew(R_des(:,1))*skew(R(:,1))+skew(R_des(:,2))*skew(R(:,2))+skew(R_des(:,3))*skew(R(:,3)));
    omg_r=L\(L.'*omg_d+K_o*ang_err);
    
    jac = robot.jacob0(q(:,k));
    qdot = pinv(jac)*[v_r;omg_r];
    qdot = scale_v(qdot,qdot_max);
    
    q(:,k+1)=q(:,k)+qdot*dt;
end