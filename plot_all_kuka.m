function plot_all_kuka(kuka_robot,t,dt,q,s,T_init,T_end)

figure;
plot(t,q.'); grid on; title 'sync axang q';
figure;
plot(t(1:end-1),diff(q,1,2).'/dt); grid on; title 'sync axang qdot';
figure;
plot(t(1:end-2),diff(q,2,2).'/(dt*dt)); grid on; title 'sync axang qddot';
T  = kuka_robot.fkine(q.');

xyz=permute(T(1:3,4,:),[1 3 2]);
figure;
plot(t,xyz'); grid on; title 'sync axang p';
dist=zeros(1,length(t));
for k=2:length(t)
    dist(k)=dist(k-1)+norm(xyz(:,k)-xyz(:,k-1));
end
figure;
plot(t,dist,t,s(1,:)); grid on; legend('dist','s'); legend off; title 'sync axang linear dist';
figure;
plot(t(1:end-1),diff(dist)/dt,t(1:end-1),diff(s(1,:),1,2)/dt); grid on; legend('v','sdot'); legend off; title 'sync axang linear v';
xyz_id=[T_init(1,4)+s(1,:)*((T_end(1,4)-T_init(1,4))/s(1,end));
        T_init(2,4)+s(1,:)*((T_end(2,4)-T_init(2,4))/s(1,end));
        T_init(3,4)+s(1,:)*((T_end(3,4)-T_init(3,4))/s(1,end))];
plot3(xyz(1,:),xyz(2,:),xyz(3,:),xyz_id(1,:),xyz_id(2,:),xyz_id(3,:)); legend('p real','p ideal'); legend off; grid on; title 'sync axang  p 3D';
try
    waitforbuttonpress
catch %#ok<*CTCH>
end
figure;
try
    kuka_robot.plot(q.','delay',dt);
catch
    disp('!! Robot plot stopped before the end !!');
end
figure;
plot_triangle(T,dt);