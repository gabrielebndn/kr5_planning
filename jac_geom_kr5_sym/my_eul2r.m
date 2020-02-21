function R=my_eul2r(eul)

cf=cos(eul(1));
sf=sin(eul(1));
ct=cos(eul(2));
st=sin(eul(2));
cp=cos(eul(3));
sp=sin(eul(3));

R = [cf*ct*cp-sf*sp, -cf*ct*sp-sf*cp, cf*st;
     sf*ct*cp+cf*sp, -sf*ct*sp+cf*cp, sf*st;
             -st*cp,           st*sp,    ct];