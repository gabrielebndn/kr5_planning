function sdot=bang_bangvstep(Ttot,T_stot,istrapezoidal,t,v_max,a_max)

dim=length(istrapezoidal);
sdot=zeros(dim,length(t));

for k=1:dim
    T_s=T_stot(k);
    T=Ttot(k);
    if istrapezoidal(k)
        rise=t<=T_s;
        coast=t>T_s&t<=T-T_s;
        dec=t>T-T_s&t<=T;
        stat=t>T;
        sdot(k,rise)=a_max(k)*t(rise);
        sdot(k,coast)=v_max(k);
        sdot(k,dec)=v_max(k)-0.5*a_max(k)*((tdec)+T_s-T);
        sdot(k,stat)=0;
    else
        rise=t<=T_s;
        stat=t>T;
        dec=~(rise|stat);
        sdot(k,rise)=a_max(k)*t(rise);
        sdot(k,dec)=a_max(k)*(2*T_s-t(dec));
        sdot(k,stat)=0;
    end
end