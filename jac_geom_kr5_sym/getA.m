function A=getA(k,dh)
% theta d a alpha

sa=round(sin(dh(k,4)));
ca=round(cos(dh(k,4)));

if dh(k,2)==0
    dk=0;
else
    dk=sym(['d' num2str(k)]);
end
if dh(k,3)==0
    ak=0;
else
    ak=sym(['a' num2str(k)]);
end

qk=sym(['q' num2str(k)]);
sk=sin(qk);
ck=cos(qk);
%syms(['s' num2str(k)],['c' num2str(k)])

A= [ck, -ca*sk,  sa*sk, ak*ck;
    sk,  ca*ck, -sa*ck, ak*sk;
     0,     sa,     ca,    dk;
     0,      0,      0,     1];

