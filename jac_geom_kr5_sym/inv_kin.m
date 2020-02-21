function q=inv_kin(T_des,q_init)

p_toll=0.005;
a_toll=1*pi/180;
max_it=2000;
K_p=0.5;
K_o=0.5;

p_toll2=p_toll*p_toll;
a_toll2=a_toll*a_toll;

x_des=T_des(1:3,4);
R_des=T_des(1:3,1:3);

q=q_init;

for k=1:max_it
    T_e = computeFullKinematic(q);
    x_e=T_e(1:3,4);
    R_e=T_e(1:3,1:3);
    
    p_err = x_des - x_e;
    
    [a_err,r_err]=tr2angvec(R_des*R_e.');
    a_err=sin(a_err);
    
    if p_err.'*p_err<=p_toll2 && a_err*a_err<=a_toll2
        break;
    end
    
    v_r=K_p*p_err;
    
    ang_err=a_err*r_err.';
    L=getL(R_des,R_e);
    omg_r=L\(K_o*ang_err);

    jac=computeFullJacobian(q);
    
    q=q+pinv(jac)*[v_r;omg_r];
end