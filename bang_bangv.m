function [sdot,T_max]=bang_bangv(L,t,v_max,a_max)

dim=length(L);
sdot=zeros(dim,length(t));
istrapezoidal=L>v_max.*v_max./a_max;
T_max=0;

for k=1:dim
    if istrapezoidal(k)
        T_s=v_max(k)/a_max(k);
        T=T_s+L(k)/v_max(k);
        rise=t<=T_s;
        coast=t>T_s&t<=T-T_s;
        dec=t>T-T_s&t<=T;
        stat=t>T;
        sdot(k,rise)=a_max(k)*t(rise);
        sdot(k,coast)=v_max(k);
        sdot(k,dec)=v_max(k)-0.5*a_max(k)*((tdec)+T_s-T);
        sdot(k,stat)=0;
    else
        T_s=sqrt(L(k)/a_max(k));
        T=2*T_s;
        rise=t<=T_s;
        stat=t>T;
        dec=~(rise|stat);
        sdot(k,rise)=a_max(k)*t(rise);
        sdot(k,dec)=a_max(k)*(2*T_s-t(dec));
        sdot(k,stat)=0;
    end
    if T>T_max, T_max=T; end
end