% Scales vectors exceeding given bounds.
% 
% [v_sc,sc] = scale_v(v,v_max,v_min) returns a vector v_sc proportional to v
% so that v_min<=v_sc<=v_max.
% 
% If it is already v_min<=v<=v_max, then v_sc=v, and sc<=1 is the minimum
% possible value so that v_min<=v/sc<=v_max.
% Otherwise v_sc=v/sc, with minimum sc>1.
%
% It is assumed v_max>0 and v_min<0.
%
% [v_sc,sc] = scale_v(v,v_max) is the same as scale_v(v,v_max,-v_max).
function [v_sc,sc] = scale_v(v,v_max,v_min)

if nargin<3
    sc=max(abs(v)./v_max);
else
    sc_max=max(v./v_max);
    sc_min=max(v./v_min);
    sc=max(sc_max,sc_min);
end

if sc>1
    v_sc=v/sc;
else
    v_sc=v;
end