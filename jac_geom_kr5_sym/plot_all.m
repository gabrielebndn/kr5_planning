outputfile='kuka_log_08_02_5_4.txt' %#ok<*NOPTS>

ouputfileID = fopen(outputfile);
A = textscan(ouputfileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(ouputfileID);
x_e   = [ A{1}, A{2}, A{3}];
x_des = [ A{4}, A{5}, A{6}];
v_des = [ A{7}, A{8}, A{9}];
p_err = [A{10},A{11},A{12}];
v_r   = [A{13},A{14},A{15}];
v_en=[0; sqrt(sum((diff(x_e,1,1)/0.012).^2,2))];
v_dn=[0; sqrt(sum((diff(x_des,1,1)/0.012).^2,2))];
t=0.012*(0:(size(x_e,1)-1));
%clear A ouputfileID

minx=min(min(x_e(:,1)),min(x_des(:,1)))-0.1;
miny=min(min(x_e(:,2)),min(x_des(:,2)))-0.1;
minz=min(min(x_e(:,3)),min(x_des(:,3)))-0.1;
maxx=max(max(x_e(:,1)),max(x_des(:,1)))+0.1;
maxy=max(max(x_e(:,2)),max(x_des(:,2)))+0.1;
maxz=max(max(x_e(:,3)),max(x_des(:,3)))+0.1;

axisvec=[minx maxx miny maxy minz maxz];

try
    for k=1:size(x_e,1)
        plot3(x_e(1:k,1),x_e(1:k,2),x_e(1:k,3),x_des(1:k,1),x_des(1:k,2),x_des(1:k,3));
        grid on; xlabel 'x-axis'; ylabel 'yaxis'; axis on; axis(axisvec);
        pause(1/60);
    end
catch err
    disp('error before!')
end