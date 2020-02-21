function [s,T_max]=bang_bang_sync(L,t,v_max,a_max)

t_l=length(t);
dim=size(L,1);
s=zeros(dim,t_l);

T=zeros(dim,1);
T_s=zeros(dim,1);
istrapezoidal=L>v_max.*v_max./a_max;
for k=1:dim
    if istrapezoidal(k)
        T_s(k)=v_max(k)/a_max(k);
        T(k)=T_s(k)+L(k)/v_max(k);
    else
        T_s(k)=sqrt(L(k)/a_max(k));
        T(k)=2*T_s(k);
    end
end
T_max=max(T);

for k=1:dim
    if T(k) < T_max
        t_sc = t*(T(k)/T_max);
    else
        t_sc = t;
    end
    if istrapezoidal(k)
        rise=t_sc<=T_s(k);
        coast=t_sc>T_s(k)&t_sc<=T(k)-T_s(k);
        dec=t_sc>T(k)-T_s(k)&t_sc<=T(k);
        stat=t_sc>T(k);
        s(k,rise)=0.5*a_max(k)*t_sc(rise).*t_sc(rise);
        s(k,coast)=0.5*a_max(k)*T_s(k)*T_s(k)+v_max(k)*(t_sc(coast)-T_s(k));
        s(k,dec)=0.5*a_max(k)*T_s(k)*T_s(k)+v_max(k)*(T(k)-2*T_s(k))+v_max(k).*(t_sc(dec)+T_s(k)-T(k))-0.5*a_max(k)*((t_sc(dec)+T_s(k)-T(k)).^2);
        s(k,stat)=L(k);
    else
        rise=t_sc<=T_s(k);
        stat=t_sc>T(k);
        dec=~(rise|stat);
        s(k,rise)=0.5*a_max(k)*t_sc(rise).*t_sc(rise);
        s(k,dec)=0.5*a_max(k)*T_s(k)*T_s(k)+a_max(k)*T_s(k)*(t_sc(dec)-T_s(k))-0.5*a_max(k)*((t_sc(dec)-T_s(k)).^2);
        s(k,stat)=L(k);
    end
end 
