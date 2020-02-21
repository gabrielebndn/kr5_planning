function [v_sc,sc]=scale(v,v_max)
    sc=max(abs(v)./v_max);
    if sc>1
        v_sc=v/sc;
    else
        v_sc=v;
        sc=0;
    end