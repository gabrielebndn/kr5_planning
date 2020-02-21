% loadfile = 'kuka_plans.mat';
loadfile = 'kuka_plans2.mat';

if ~(exist('immediate_plot','var')&&immediate_plot)
    clearvars -except loadfile
    fprintf(['loading: ',loadfile,'\n\n']);
    load(loadfile);
else
    clear loadfile
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('plotting joint trajectories...');

disp('joints interpolation...');
figure;
plot(q_joints.');
title 'joints interpolation'

disp('euler interpolation...');
figure;
plot(q_eul.');
title 'euler interpolation'

disp('rpy interpolation...');
figure;
plot(q_rpy.');
title 'rpy interpolation'

disp('axis-angle interpolation...');
figure;
plot(q_axang.');
title 'axis-angle interpolation'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% position plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\nplotting end-effector trajectories...\n');

disp('joints interpolation path...');
figure;
plot(permute(T_joints(1:3,4,:),[3 1 2]));
title 'joints interpolation path'

disp('euler interpolation path...');
figure;
plot(permute(T_eul(1:3,4,:),[3 1 2]));
title 'euler interpolation path'

disp('rpy interpolation path...');
figure;
plot(permute(T_rpy(1:3,4,:),[3 1 2]));
title 'rpy interpolation path'

disp('axis-angle interpolation path...');
figure;
plot(permute(T_axang(1:3,4,:),[3 1 2]));
title 'axis-angle interpolation path'

fprintf('press key to continue...\n\n');
while 1
    button = waitforbuttonpress;
    if button==1
        break;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% animations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('displaying animations...');
figure;
kuka_robot.plot(q_joints(:,1).');

disp('joints interpolation animation...');
title 'joints interpolation animation'
kuka_robot.plot(q_joints.');

disp('euler interpolation animation...');
title 'euler interpolation animation'
kuka_robot.plot(q_eul.');

disp('rpy interpolation animation...');
title 'rpy interpolation animation'
kuka_robot.plot(q_rpy.');

disp('axis-angle interpolation animation...');
title 'axis-angle interpolation animation'
kuka_robot.plot(q_axang.');