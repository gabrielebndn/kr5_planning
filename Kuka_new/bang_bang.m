function s=bang_bang(L,t,v_max,a_max)

dim=length(L);
s=zeros(dim,1);

for k=1:dim
    istrapezoidal=L(k)>v_max(k).*v_max(k)./a_max(k);
    if istrapezoidal
        T_s=v_max(k)/a_max(k);
        T=(L(k)*a_max(k)+v_max(k)*v_max(k))/(a_max(k)*v_max(k));

        if t<=T_s     	% rise
            s(k)=0.5*a_max(k)*t.*t;
        elseif t<=T-T_s % coast
            s(k)=0.5*a_max(k)*T_s*T_s+v_max(k)*(t-T_s);
        elseif t<=T     % desc
            s(k)=0.5*a_max(k)*T_s*T_s+v_max(k)*(T-2*T_s)+v_max(k).*(t+T_s-T)-0.5*a_max(k)*((t+T_s-T).^2);
        else            % stat
            s(k)=L(k);
        end
    else
        T_s=sqrt(L(k)/a_max(k));
        T=2*T_s;
        if t<=T_s       % rise
            s(k)=0.5*a_max(k)*t.*t;
        elseif t<=T     % desc
            s(k)=0.5*a_max(k)*T_s*T_s+a_max(k)*T_s*(t-T_s)-0.5*a_max(k)*((t-T_s).^2);
        else            % stat
            s(k)=L(k);
        end
    end
end