function A_R=getAR(k,dh)

sa=round(sin(dh(k,4)));
ca=round(cos(dh(k,4)));

qk=sym(['q' num2str(k)]);
sk=sin(qk);
ck=cos(qk);
%syms(['s' num2str(k)],['c' num2str(k)])

A_R= [ck, -ca*sk,  sa*sk;
      sk,  ca*ck, -sa*ck;
       0,     sa,     ca];
