function s=multi_bang_bang(L,t,v_max,a_max)

dim=length(L);
s=zeros(dim,length(t));

for k=1:dim
    istrapezoidal=L(k)>v_max(k).*v_max(k)./a_max(k);
    if istrapezoidal
        T_s=v_max(k)/a_max(k);
        T=(L(k)*a_max(k)+v_max(k)*v_max(k))/(a_max(k)*v_max(k));
        rise=t<T_s;
        coast=t>=T_s&t<=T-T_s;
        dec=t>T-T_s&t<=T;
        stat=t>T;
        s(k,rise)=0.5*a_max(k)*t(rise).*t(rise);
        s(k,coast)=0.5*a_max(k)*T_s*T_s+v_max(k)*(t(coast)-T_s);
        s(k,dec)=0.5*a_max(k)*T_s*T_s+v_max(k)*(T-2*T_s)+v_max(k).*(t(dec)+T_s-T)-0.5*a_max(k)*((t(dec)+T_s-T).^2);
        s(k,stat)=L(k);
    else
        T_s=sqrt(L(k)/a_max(k));
        T=2*T_s;
        rise=t<T_s;
        dec=t>=T_s&t<=T;
        stat=t>T;
        s(k,rise)=0.5*a_max(k)*t(rise).*t(rise);
        s(k,dec)=0.5*a_max(k)*T_s*T_s+a_max(k)*T_s*(t(dec)-T_s)-0.5*a_max(k)*((t(dec)-T_s).^2);
        s(k,stat)=L(k);
    end
end