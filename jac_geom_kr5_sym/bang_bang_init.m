function [T,T_s]=bang_bang_init(L,sdot_max,sddot_max)
    if L>sdot_max*sdot_max/sddot_max
        T_s=sdot_max/sddot_max;
        T=T_s+L/sdot_max;
    else
        T_s=sqrt(L/sddot_max);
        T=2*T_s;
    end