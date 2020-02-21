dt=0.01;
T=10;
t=0:dt:T;

d_3=0.4;
d_5=0.4;

kuka_robot=SerialLink(... % theta d a alpha
    [0	0	0    pi/2;
     0  0	0   -pi/2;
     0  d_3 0	-pi/2;
     0	0	0    pi/2;
     0	d_5	0    pi/2;
     0	0	0   -pi/2;
     0	0	0.2       0],...
     'name','kuka');

q_init=zeros(7,1).' %#ok<NOPTS>
q_end=pi/2*ones(7,1).' %#ok<NOPTS>
q_init=q_init.';
q_end=q_end.';
T_init=kuka_robot.fkine(q_init) %#ok<NOPTS>
T_end=kuka_robot.fkine(q_end) %#ok<NOPTS>

v_max=[pi/2;pi;pi;pi;2*pi;2*pi;2*pi].' %#ok<NOPTS>
a_max=2*v_max %#ok<NOPTS>

v_max=v_max.';
a_max=a_max.';

vlin_max=2 %#ok<NOPTS>
alin_max=4 %#ok<NOPTS>

omg_max=2 %#ok<NOPTS>
omgdot_max=4 %#ok<NOPTS>

v_eulmax=omg_max %#ok<NOPTS>
a_eulmax=omgdot_max %#ok<NOPTS>


% disp('async...')
% tic;
% q=plan_joints_async(q_init,q_end,t,v_max,a_max);
% toc;
% 
% figure;
% plot(t,q.'); title 'async pos';
% figure;
% plot(t(1:end-1),diff(q,1,2).'/dt); title 'async vel';
% figure;
% plot(t(1:end-2),diff(q,2,2).'/(dt*dt)); title 'async acc';
% try
%     kuka_robot.plot(q.','delay',dt);
% catch err
%     disp('!! Plot stopped before the end !!');
% end
% figure;
% plot_triangle(T,dt);
%
%
% disp('sync...')
% tic;
% q=plan_joints_sync(q_init,q_end,t,v_max,a_max);
% toc;
% 
% figure;
% plot(t,q.'); title 'sync pos';
% figure;
% plot(t(1:end-1),diff(q,1,2).'/dt); title 'sync vel';
% figure;
% plot(t(1:end-2),diff(q,2,2).'/(dt*dt)); title 'sync acc';
% figure;
% try
%     kuka_robot.plot(q.','delay',dt);
% catch err
%     disp('!! Plot stopped before the end !!');
% end
% figure;
% plot_triangle(T,dt);


disp('axang...')
tic;
[q,s]=plan_lin_axang(kuka_robot,T_init,T_end,q_init,t,[vlin_max;omg_max],[alin_max;omgdot_max],v_max);
toc;

%q=plan_lin_eul(kuka_robot,T_init(1:3,4),[T_end(1:3,4);tr2eul(T_end).'],q_init,0,t,vlin_max,alin_max);
%toc;

plot_all_kuka(kuka_robot,t,dt,q,s,T_init,T_end);

%plot_all_kuka(kuka_robot,t,dt,q,zeros(length(t),1),T_init,T_end);