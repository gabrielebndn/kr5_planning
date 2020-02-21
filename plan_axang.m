function q=plan_axang(robot,q_init,T_init,T_end,t,v_max,a_max)

%dt=t(2)-t(1);
p_init = T_init(1:3,4);
p_end = T_end(1:3,4);
p_diff=p_end-p_init;
p_n=norm(p_diff);
%p_ver=p_diff/p_n;
R_init = T_init(1:3,1:3);
R_ie = R_init.'*T_end(1:3,1:3);
[theta_f,r]  =tr2angvec(R_ie);
s = bang_bang_sync([p_n;theta_f],t,v_max,a_max);

% s_d=diff(s,1,2)./dt;
% q=zeros(length(q_init),length(t));
% q(:,1)=q_init;
% for k=2:length(t)
%     jac=robot.jacob0(q(:,k-1));
%     v_d=[p_ver*s_d(1,k-1);r.'*s_d(2,k-1)];
%     qdot=pinv(jac)*v_d;
%     q(:,k)=q(:,k-1)+qdot*dt;
% end

p = repmat(p_init,1,length(t)) + repmat(sign(p_end-p_init),1,length(t)).*repmat(s(1,:),3,1);
q=zeros(length(q_init),length(t));
q(:,1)=q_init;
T=zeros(4);
T(4,4)=1;
for k=2:length(t)
    T(1:3,1:3) = R_init*angvec2r(s(2,k),r);
    T(1:3,4) = p(:,k);
    q(:,k)=robot.ikine(T,q(:,k-1),'ilimit',10000);
end
