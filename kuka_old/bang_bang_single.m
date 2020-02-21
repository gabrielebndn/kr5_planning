function s=bang_bang(L,t,v_max,a_max)

t_l=length(t);
s=zeros(1,t_l);
s(1)=0;

istrapezoidal=L>v_max.*v_max./a_max;
if istrapezoidal
    T_s=v_max/a_max;
    T=(L*a_max+v_max*v_max)/(a_max*v_max);
    rise=t<T_s;
    coast=t>=T_s&t<=T-T_s;
    dec=t>T-T_s&t<=T;
    stat=t>T;
    s(rise)=0.5*a_max*t(rise).*t(rise);
    s(coast)=0.5*a_max*T_s*T_s+v_max*(t(coast)-T_s);
    s(dec)=0.5*a_max*T_s*T_s+v_max*(T-2*T_s)+v_max.*(t(dec)+T_s-T)-0.5*a_max*((t(dec)+T_s-T).^2);
    s(stat)=L;
else
    T_s=sqrt(L/a_max);
    T=2*T_s;
    rise=t<T_s;
    dec=t>=T_s&t<=T;
    stat=t>T;
    s(rise)=0.5*a_max*t(rise).*t(rise);
    s(dec)=0.5*a_max*T_s*T_s+a_max*T_s*(t(dec)-T_s)-0.5*a_max*((t(dec)-T_s).^2);
    s(stat)=L;
end
