function [T,T_s,istrapezoidal]=bang_bang_vinit(L,v_max,a_max)

dim=length(L);
istrapezoidal=L>v_max.*v_max./a_max;

T=zeros(dim,1);
T_s=zeros(dim,1);

T_s(istrapezoidal)=v_max(istrapezoidal)/a_max(istrapezoidal);
T(istrapezoidal)=T_s(istrapezoidal)+L(istrapezoidal)/v_max(istrapezoidal);

T_s(~istrapezoidal)=sqrt(L(~istrapezoidal)/a_max(~istrapezoidal));
T(~istrapezoidal)=2*T_s(~istrapezoidal);