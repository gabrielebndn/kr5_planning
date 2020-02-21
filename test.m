dt=0.01;
T=10;
t=0:dt:T;

v_max=[pi/2;pi;pi;pi;2*pi;2*pi;2*pi];
a_max=2*v_max;

disp('async...')
disp('total:')
tic;
q=plan_joints_async(zeros(7,1),pi/2*ones(7,1),t,v_max,a_max);
toc;
disp('one by one:')
tic;
q_p=zeros(7,length(t));
for k=1:length(t);
    q_p(:,k)=plan_joints_async(zeros(7,1),pi/2*ones(7,1),t(k),v_max,a_max);
end
toc;
disp('difference:')
max_diff=max(abs(q-q_p).');
disp(max_diff)
fprintf('\n');

disp('sync...')
disp('total:')
tic;
q=plan_joints_sync(zeros(7,1),pi/2*ones(7,1),t,v_max,a_max);
toc;
disp('one by one:')
tic;
q_p=zeros(7,length(t));
for k=1:length(t);
    q_p(:,k)=plan_joints_sync(zeros(7,1),pi/2*ones(7,1),t(k),v_max,a_max);
end
toc;
disp('difference:')
max_diff=max(abs(q-q_p).');
disp(max_diff)
fprintf('\n');