function [s,sdot]=bang_bang_calc(T,T_s,t,sddot_max)
    if(t<=T_s)
        sdot=sddot_max*t;
        s=0.5*sdot*t;
    elseif(t<T-T_s)
        sdot=sddot_max*T_s;
        s=sdot*(t-0.5*T_s);
    elseif(t<T)
        sdot=sddot_max*(T-t);
        L=sddot_max*T_s*(T-T_s);
        s=L-0.5*sdot*(T-t);
    else
        sdot=0;
        s=sddot_max*T_s*(T-T_s);
    end