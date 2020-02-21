function J=my_eul2jac(eul)

sp=sin(eul(1));
cp=cos(eul(1));
st=sin(eul(2));
ct=cos(eul(2));

J = [ 0, -sp, cp*st;
      0,  cp, sp*st;
      1,   0,    ct];