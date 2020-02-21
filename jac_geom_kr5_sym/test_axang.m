%using angle-axis
%#ok<*NOPTS>
q_init=[0;pi/2;pi/2;0;-pi/2;0];

T_init=computeFullKinematic(q_init);
R_init=T_init(1:3,1:3)

% rot_d = [0 0 1;
%          1 0 0;
%          0 1 0];
% rot_d = [-1 0  0;
%           0 1  0;
%           0 0 -1];
% R_d=rot_d*R_init;
% R_d=eye(3);
% R_d(1,1)=-1;
% R_d(2,2)=-1;
% R_d=[1 0 0; 0 0 1; 0 -1 0];
% R_d = [-1 0 0; 0 1 0; 0 0 -1];
R_d = [0 0 -1; 0 -1 0; -1 0 0]


va_max=pi/4;
aa_max=pi/8;

t_tot=10;
dt=0.012;
N=ceil(t_tot/dt);

T=zeros(3,3,N);
t=zeros(N,1);
for k=1:N
    tk=(k-1)*dt;
    [th,r]=tr2angvec(R_init.'*R_d);
    r=r.';
    [Time_a,Time_as] = bang_bang_init(abs(th),va_max,aa_max);
    [st,stdot]=bang_bang_calc(Time_a,Time_as,tk,aa_max);
    th_des=st*sign(th);
    R_des=R_init*angvec2r(th_des,r);
    T(:,:,k)=R_des;
    t(k)=tk;
end
    
figure;
plot_triangle(T(:,:,end),'actual orientation',0.012);
figure;
plot_triangle(T,'actual orientation',0.012);
disp('actual orientation over');