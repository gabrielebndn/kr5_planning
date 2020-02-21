%#ok<*NOPTS>
%#ok<*CTCH>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOADING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

basefolder = 'kr5_lastlogs_15_12_14'
file='kuka_log_zp.txt'
inputfile = [basefolder '/' file];
% inputfile = '~/Desktop/kuka_log_zp.txt'
has_z=0;
if strcmp(file,'kuka_log_t6.txt')||strcmp(file,'kuka_log_zp.txt')||strcmp(file,'kuka_log_ap.txt')
    has_z=1;
end

delta_t=0.012;

inputfileID = fopen(inputfile);
format_string = '%f';
for k=1:26
    format_string = [format_string ' %f']; %#ok<AGROW>
end
if has_z
    format_string = [format_string ' %f %f %f %f %f %f %f %f %f'];
end
A = textscan(inputfileID,format_string);
fclose(inputfileID);
x_e   = [ A{1}, A{2}, A{3}];
x_des = [ A{4}, A{5}, A{6}];
v_des = [ A{7}, A{8}, A{9}];
p_err = [A{10},A{11},A{12}];
v_r   = [A{13},A{14},A{15}];
q_e = [A{16},A{17},A{18},A{19},A{20},A{21}];
q_des = [A{22},A{23},A{24},A{25},A{26},A{27}];
if has_z
    zyz_e = [A{28},A{29},A{30}];
    zyz_des = [A{31},A{32},A{33}];
    ang_err = [A{34},A{35},A{36}];
end
v_en=[0; sqrt(sum((diff(x_e,1,1)/0.012).^2,2))];
v_dn=[0; sqrt(sum((diff(x_des,1,1)/0.012).^2,2))];
N=size(x_e,1);
t=0.012*(0:(N-1));

T_e=zeros(4,4,N);
for k=1:N
    T_e(:,:,k)=computeFullKinematic(q_e(k,:));
end
if ~has_z
    zyz_e=zeros(N,3);
    for k=1:N
        zyz_e(k,:)=my_tr2eul(T_e(:,:,k));
    end
end

clear A inutfileID

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PRINTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('x real init:');
disp(x_e(1,:))
disp('x des init:');
disp(x_des(1,:))
disp('T real init:');
disp(T_e(:,:,1))

disp('x real end:');
disp(x_e(end,:))
disp('x des end:');
disp(x_des(end,:))
disp('T real end:');
disp(T_e(:,:,end))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DISPLAYING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

minx=min(min(x_e(:,1)),min(x_des(:,1)))-0.1;
miny=min(min(x_e(:,2)),min(x_des(:,2)))-0.1;
minz=min(min(x_e(:,3)),min(x_des(:,3)))-0.1;
maxx=max(max(x_e(:,1)),max(x_des(:,1)))+0.1;
maxy=max(max(x_e(:,2)),max(x_des(:,2)))+0.1;
maxz=max(max(x_e(:,3)),max(x_des(:,3)))+0.1;

axisvec=[minx maxx miny maxy minz maxz];

figure;
plot3(x_e(:,1),x_e(:,2),x_e(:,3),x_des(:,1),x_des(:,2),x_des(:,3));
grid on; xlabel 'x-axis'; ylabel 'yaxis'; axis on; axis(axisvec);
save_h = figure;
try
    for k=1:size(x_e,1)
        if gcf~=save_h
            disp('plot stopped before the end')
            break;
        end
        plot3(x_e(1:k,1),x_e(1:k,2),x_e(1:k,3),x_des(1:k,1),x_des(1:k,2),x_des(1:k,3));
        grid on; xlabel 'x-axis'; ylabel 'yaxis'; axis on; axis(axisvec);
        pause(delta_t);
    end
    close(save_h);
catch
    disp('plot stopped before the end')
end
clear save_h

disp('plotting orientation');
figure;
plot_triangle(T_e,'actual orientation',delta_t);
disp('orientation over');

try
    d_1=0.132;
    d_4=0.295;
    d_6=0.080;

    a_1=0.075;
    a_2=0.270;
    a_3=0.090;

    h=1.023;
    d_1=d_1+h;
    
    END_EFF=0.09;
    
    d_6=d_6+END_EFF;

    kr5=SerialLink(... % theta d a alpha
        [0	d_1	a_1   pi/2;
         0  0	a_2      0;
         0  0   a_3   pi/2;
         0	d_4	0    -pi/2;
         0	0	0     pi/2;
         0	d_6	0        0],...
         'name','kuka\_kr5');
    figure;
    kr5.plot(q_e,'delay',delta_t)
catch
    disp('!! Plot error before the end !!');
end