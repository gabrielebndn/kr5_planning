function [s,sdot]=bang_bang(L,t,sdot_max,sddot_max)
    if L>sdot_max*sdot_max/sddot_max
        T_s=sdot_max/sddot_max;
        T=T_s+L/sdot_max;        
    else
        T_s=sqrt(L/sddot_max);
        T=2*T_s;
    end
    
    if(t<=T_s)
        sdot=sddot_max*t;
        s=0.5*sdot*t;
    elseif(t<T-T_s)
        sdot=sddot_max*T_s;
        s=sdot*(t-0.5*T_s);
    elseif(t<T)
        sdot=sddot_max*(T-t);
        s=L-0.5*sdot*(T-t);
    else
        sdot=0;
        s=L;
    end