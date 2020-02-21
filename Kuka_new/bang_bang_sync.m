function s=bang_bang_sync(L,t,v_max,a_max)

dim=size(L,1);
s=zeros(dim,1);

T_tot=zeros(dim,1);
T_tot_s=zeros(dim,1);
istrapezoidal=false(dim,1);
for k_d=1:dim
    istrapezoidal(k_d)=L(k_d)>v_max(k_d).*v_max(k_d)./a_max(k_d);
    if istrapezoidal(k_d)
        T_tot_s(k_d)=v_max(k_d)/a_max(k_d);
        T_tot(k_d)=(L(k_d)*a_max(k_d)+v_max(k_d)*v_max(k_d))/(a_max(k_d)*v_max(k_d));
    else
        T_tot_s(k_d)=sqrt(L(k_d)/a_max(k_d));
        T_tot(k_d)=2*T_tot_s(k_d);
    end
end
T_max=max(T_tot);

for k_d=1:dim
    if T_tot(k_d) < T_max
        t_sc = t*(T_tot(k_d)/T_max);
    else
        t_sc = t;
    end
    if istrapezoidal(k_d)
        T_s=v_max(k_d)/a_max(k_d);
        T=(L(k_d)*a_max(k_d)+v_max(k_d)*v_max(k_d))/(a_max(k_d)*v_max(k_d));
        if t_sc<=T_s        % rise
            s(k_d)=0.5*a_max(k_d)*t_sc*t_sc;
        elseif t_sc<=T-T_s  % coast
            s(k_d)=0.5*a_max(k_d)*T_s*T_s+v_max(k_d)*(t_sc-T_s);
        elseif  t_sc<=T     % desc
            s(k_d)=0.5*a_max(k_d)*T_s*T_s+v_max(k_d)*(T-2*T_s)+v_max(k_d).*(t_sc+T_s-T)-0.5*a_max(k_d)*((t_sc+T_s-T).^2);
        else                % stat
            s(k_d)=L(k_d);
        end
    else
        T_s=sqrt(L(k_d)/a_max(k_d));
        T=2*T_s;
        rise=t_sc<T_s;
        dec=t_sc>=T_s&t_sc<=T;
        stat=t_sc>T;
        if t_sc<=T_s        % rise
            s(k_d)=0.5*a_max(k_d)*t_sc.*t_sc;
        elseif  t_sc<=T     % desc
            s(k_d)=0.5*a_max(k_d)*T_s*T_s+a_max(k_d)*T_s*(t_sc-T_s)-0.5*a_max(k_d)*((t_sc-T_s).^2);
        else                % stat       
            s(k_d,stat)=L(k_d);
        end
    end
end 
