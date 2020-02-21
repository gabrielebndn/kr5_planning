function q=plan_joints_sync(~,~,~,q_init,q_end,t,~,v_max,a_max)

t_l=length(t);
n_j=length(q_init);
q=zeros(n_j,t_l);
q(:,1)=q_init;

T_tot=zeros(n_j,1);
T_tot_s=zeros(n_j,1);
istrapezoidal=false(n_j,1);
for k_j=1:n_j
    L=abs(q_end(k_j)-q_init(k_j));
    istrapezoidal(k_j)=L>v_max(k_j).*v_max(k_j)./a_max(k_j);
    if istrapezoidal(k_j)
        T_tot_s(k_j)=v_max(k_j)/a_max(k_j);
        T_tot(k_j)=(L*a_max(k_j)+v_max(k_j)*v_max(k_j))/(a_max(k_j)*v_max(k_j));
    else
        T_tot_s(k_j)=sqrt(L/a_max(k_j));
        T_tot(k_j)=2*T_tot_s(k_j);
    end
end
T_max=max(T_tot);

for k_j=1:n_j
    L=abs(q_end(k_j)-q_init(k_j));
    if T_tot(k_j) < T_max
        t_sc = t*(T_tot(k_j)/T_max);
    else
        t_sc = t;
    end
    sgn=sign(q_end(k_j)-q_init(k_j));
    if istrapezoidal(k_j)
        T_s=v_max(k_j)/a_max(k_j);
        T=(L*a_max(k_j)+v_max(k_j)*v_max(k_j))/(a_max(k_j)*v_max(k_j));
        rise=t_sc<T_s;
        coast=t_sc>=T_s&t_sc<=T-T_s;
        dec=t_sc>T-T_s&t_sc<=T;
        stat=t_sc>T;
        q(k_j,rise)=q_init(k_j)+0.5*a_max(k_j)*sgn*t_sc(rise).*t_sc(rise);
        q(k_j,coast)=q_init(k_j)+0.5*a_max(k_j)*sgn*T_s*T_s+v_max(k_j)*sgn.*(t_sc(coast)-T_s);
        q(k_j,dec)=q_init(k_j)+0.5*a_max(k_j)*sgn*T_s*T_s+v_max(k_j)*sgn.*(T-2*T_s)+v_max(k_j).*sgn*(t_sc(dec)+T_s-T)-0.5*a_max(k_j)*sgn*((t_sc(dec)+T_s-T).^2);
        q(k_j,stat)=q_end(k_j);
    else
        T_s=sqrt(L/a_max(k_j));
        T=2*T_s;
        rise=t_sc<T_s;
        dec=t_sc>=T_s&t_sc<=T;
        stat=t_sc>T;
        q(k_j,rise)=q_init(k_j)+0.5*a_max(k_j)*sgn*t_sc(rise).*t_sc(rise);
        q(k_j,dec)=q_init(k_j)+0.5*a_max(k_j)*sgn*T_s*T_s+a_max(k_j)*T_s*sgn.*(t_sc(dec)-T_s)-0.5*a_max(k_j)*sgn*((t_sc(dec)-T_s).^2);
        q(k_j,stat)=q_end(k_j);
    end
end 
