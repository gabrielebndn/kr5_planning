function eul = my_tr2eul(T)

theta=atan2(sqrt(T(1,3)*T(1,3)+T(2,3)*T(2,3)),T(3,3));

%if theta>eps
if theta~=0
    phi=atan2(T(2,3),T(1,3));
    psi=atan2(T(3,2),-T(3,1));
else
    phi=0;
    psi=atan2(T(1,1),T(2,1));
end
    

eul=[phi theta psi];

