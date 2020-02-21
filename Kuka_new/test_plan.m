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

v_max=[pi/2;pi;pi;pi;2*pi;2*pi;2*pi];
a_max=2*v_max;

disp('async...')
tic;
q=plan_joints_async(zeros(7,1),pi/2*ones(7,1),t,v_max,a_max);
toc;

figure;
plot(t,q.'); title 'async pos';
figure;
plot(t(1:end-1),diff(q,1,2).'/dt); title 'async vel';
figure;
plot(t(1:end-2),diff(q,2,2).'/(dt*dt)); title 'async acc';

disp('sync...')
tic;
q=plan_joints_sync(zeros(7,1),pi/2*ones(7,1),t,v_max,a_max);
toc;

figure;
plot(t,q.'); title 'sync pos';
figure;
plot(t(1:end-1),diff(q,1,2).'/dt); title 'sync vel';
figure;
plot(t(1:end-2),diff(q,2,2).'/(dt*dt)); title 'sync acc';