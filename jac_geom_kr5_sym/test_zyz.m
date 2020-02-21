%using euler angles
%#ok<*NOPTS>
q_init=[0;pi/2;pi/2;0;-pi/2;0];

T_init=computeFullKinematic(q_init);
R_init=T_init(1:3,1:3)
zyz_init=tr2eul(T_init(1:3,1:3))

% rot_d = [0 0 1;
%          1 0 0;
%          0 1 0];
% rot_d = [-1 0  0;
%           0 1  0;
%           0 0 -1];
% R_d=rot_d*R_init;
% R_d=eye(3);
% R_d=[1 0 0; 0 0 1; 0 -1 0];
% R_d=eye(3); R_d(1,1)=-1; R_d(2,2)=-1;
% R_d = [-1 0 0; 0 1 0; 0 0 -1];
R_d = [0 0 -1; 0 -1 0; -1 0 0]
zyz_d=zyz_init+wrapToPi(tr2eul(R_d)-zyz_init)
% zyz_d=[pi pi pi]
% R_d=my_eul2r(zyz_d);

va_max=pi/4;
aa_max=pi/8;

t_tot=10;
dt=0.012;
N=ceil(t_tot/dt);

T=zeros(3,3,N);
t=zeros(N,1);
zyz_tot=zeros(3,N);
for k=1:N
    tk=(k-1)*dt;
    L_zyz=norm(zyz_d-zyz_init);
    [Time_a,Time_as] = bang_bang_init(L_zyz,va_max,aa_max);
    [st,stdot]=bang_bang_calc(Time_a,Time_as,tk,aa_max);
    zyz_des=zyz_init+st/L_zyz*(zyz_d-zyz_init);
    T(:,:,k)=my_eul2r(zyz_des);
    t(k)=tk;
    zyz_tot(:,k)=wrapToPi(zyz_des);
end
    
figure;
plot_triangle(T(:,:,end),'actual orientation',0.012);
figure;
plot_triangle(T,'actual orientation',0.012);
disp('actual orientation over');