function J_inv=my_eul2jac_inv(eul)

phi=eul(1);
theta=eul(2);


J_inv = ...
    [ -cos(phi)*cot(theta), -cot(theta)*sin(phi), 1;
                 -sin(phi),             cos(phi), 0;
       cos(phi)/sin(theta),  sin(phi)/sin(theta), 0];