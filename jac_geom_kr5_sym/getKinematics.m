function T=getKinematics(dh)

if nargin<1
    disp('note: using default DH:');
%     d1=0.132;
%     d4=0.295;
%     d6=0.080;
% 
%     a1=0.075;
%     a2=0.270;
%     a3=0.090;
    syms d1 d4 d6 a1 a2 a3
    dh=[ 0	d1	a1   pi/2;
         0  0	a2      0;
         0  0   a3   pi/2;
         0	d4	0   -pi/2;
         0	0	0    pi/2;
         0	d6	0       0] %#ok<NOPRT>
    fprintf('\n\n');
end

T=sym(eye(4));
for k=1:6
    T=simplify(T*getA(k,dh));
end