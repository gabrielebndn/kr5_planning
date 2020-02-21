clear all

%savefile = 'kuka_plans.mat';
%savefile = 'kuka_plans2.mat';
savefile = 'kuka_plans3.mat';

fprintf(['Started. Ouptut will be saved to: ',savefile,'\n\n']);
immediate_plot = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% defining robot %%%%%%%%%%%%%%%%%%%%%%%

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
 
n_steps=100;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% defining initial and end point %%%%%%%%

fprintf('Starting...\n\n');

p_init=[0.1;0.1;0.1];
R_init=eye(3);
T_init=[R_init,p_init;0 0 0 1];

p_end=[-0.3;0.2;-0.1];
%R_end=[0 1 0;-1 0 0;0 0 1];
R_end=rpy2r([pi/3 -pi/3 pi/4]);
T_end=[R_end,p_end;0 0 0 1];

disp('initial and final points inversion...');
tic;
q_init=kuka_robot.ikine(T_init,'ilimit',10000).';
q_end=kuka_robot.ikine(T_end,'ilimit',10000);
%q_end=kuka_robot.ikine(T_end,'ilimit',100000,'pinv','verbose','tol',1e-4).';
toc;
fprintf('\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% joints interpolation %%%%%%%%%%%%%%%%%%%
disp('angle interpolation...');
tic;
q_joints = multi_linspace(q_init,q_end,n_steps);
toc;
fprintf('\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% position interpolation %%%%%%%%%%%%%%%%%%
disp('position interpolation...');
tic;
p_cart = multi_linspace(p_init,p_end,n_steps);
toc;
fprintf('\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% euler interpolation %%%%%%%%%%%%%%%%%%%%%
disp('euler interpolation...');
tic;
eul_init = tr2eul(T_init).';
eul_end  = tr2eul(T_end).';
eul = multi_linspace(eul_init,eul_end,n_steps);
q_eul=zeros(7,n_steps);
q_k=q_init;
for k=1:n_steps
    R_k=eul2r(eul(:,k).');
    p_k=p_cart(:,k);
    T_k=[R_k,p_k;0 0 0 1];
    q_k=kuka_robot.ikine(T_k,q_k,'ilimit',10000).';
    q_eul(:,k)=q_k;
    if(mod(k,10)==0)
        fprintf('.');
    end
end
fprintf('\n');
toc;
fprintf('\n\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% rpy interpolation %%%%%%%%%%%%%%%%%%%%%%%
disp('rpy interpolation...');
tic;
rpy_init = tr2rpy(T_init).';
rpy_end  = tr2rpy(T_end).';
rpy = multi_linspace(rpy_init,rpy_end,n_steps);
q_rpy=zeros(7,n_steps);
q_k=q_init;
for k=1:n_steps
    R_k=rpy2r(rpy(:,k).');
    p_k=p_cart(:,k);
    T_k=[R_k,p_k;0 0 0 1];
    q_k=kuka_robot.ikine(T_k,q_k,'ilimit',10000).';
    q_rpy(:,k)=q_k;
    if(mod(k,10)==0)
        fprintf('.');
    end
end
fprintf('\n');
toc;
fprintf('\n\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% axis-angle interpolation %%%%%%%%%%%%%%%%%
disp('axis-angle interpolation...');
tic;
R_AB=R_init.'*R_end;
[theta_AB,v_AB]=tr2angvec(R_AB);
theta = linspace(0,theta_AB,n_steps);
q_axang=zeros(7,n_steps);
q_k=q_init;
for k=1:n_steps
    R_k=R_init*angvec2r(theta(k),v_AB);
    p_k=p_cart(:,k);
    T_k=[R_k,p_k;0 0 0 1];
    q_k=kuka_robot.ikine(T_k,q_k,'ilimit',10000).';
    q_axang(:,k)=q_k;
    if(mod(k,10)==0)
        fprintf('.');
    end
end
fprintf('\n');
toc;
fprintf('\n\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% forward kinematics %%%%%%%%%%%%%%%%%%%%%%%%
disp('computing forward kinematics...');
T_joints = kuka_robot.fkine(q_joints.');
T_eul  	 = kuka_robot.fkine(q_eul.');
T_rpy    = kuka_robot.fkine(q_rpy.');
T_axang  = kuka_robot.fkine(q_axang.');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% saving and plotting %%%%%%%%%%%%%%%%%%%%%%%
fprintf(['end!! saving to: ',savefile,'\n\n']);
save(savefile,'kuka_robot','p_init','p_end','q_joints','p_cart','q_eul','q_rpy','q_axang','T_joints','T_eul','T_rpy','T_axang')

if immediate_plot
    kuka_plot
end