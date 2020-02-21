function q=plan_joints_async(~,~,~,q_init,q_end,t,~,v_max,a_max)

t_l=length(t);
n_j=length(q_init);
q=zeros(n_j,t_l);
q(:,1)=q_init;
for k_j=1:n_j
    L=abs(q_end(k_j)-q_init(k_j));
    istrapezoidal=L>v_max(k_j).*v_max(k_j)./a_max(k_j);
    sgn=sign(q_end(k_j)-q_init(k_j));
    if istrapezoidal
        T_s=v_max(k_j)/a_max(k_j);
        T=(L*a_max(k_j)+v_max(k_j)*v_max(k_j))/(a_max(k_j)*v_max(k_j));
        rise=t<T_s;
        coast=t>=T_s&t<=T-T_s;
        dec=t>T-T_s&t<=T;
        stat=t>T;
        q(k_j,rise)=q_init(k_j)+0.5*a_max(k_j)*sgn*t(rise).*t(rise);
        q(k_j,coast)=q_init(k_j)+0.5*a_max(k_j)*sgn*T_s*T_s+v_max(k_j)*sgn.*(t(coast)-T_s);
        q(k_j,dec)=q_init(k_j)+0.5*a_max(k_j)*sgn*T_s*T_s+v_max(k_j)*sgn.*(T-2*T_s)+v_max(k_j).*sgn*(t(dec)+T_s-T)-0.5*a_max(k_j)*sgn*((t(dec)+T_s-T).^2);
        q(k_j,stat)=q_end(k_j);
    else
        T_s=sqrt(L/a_max(k_j));
        T=2*T_s;
        rise=t<T_s;
        dec=t>=T_s&t<=T;
        stat=t>T;
        q(k_j,rise)=q_init(k_j)+0.5*a_max(k_j)*sgn*t(rise).*t(rise);
        q(k_j,dec)=q_init(k_j)+0.5*a_max(k_j)*sgn*T_s*T_s+a_max(k_j)*T_s*sgn.*(t(dec)-T_s)-0.5*a_max(k_j)*sgn*((t(dec)-T_s).^2);
        q(k_j,stat)=q_end(k_j);
    end
end 
