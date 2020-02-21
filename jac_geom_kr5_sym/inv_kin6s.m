function theta = inv_kin6s(T)
%     a1=0.075; a2=0.27; a3=0.09;
%     d1=0.132; d4=0.295; d6=0.08;
    h=1.023; %altezza base

    a2 = 0.27;
    a3 = 0.09;

    d3 = 0;
    d4 = 0.295;
    
    T(3,4)=T(3,4)-h-0.132;
    
    theta=zeros(6,1);

    % The following parameters are extracted from the Homogeneous 
    % Transformation as defined in equation 1, p. 34

    Ox = T(1,2);
    Oy = T(2,2);
    Oz = T(3,2);

    Ax = T(1,3);
    Ay = T(2,3);
    Az = T(3,3);

    Px = T(1,4);
    Py = T(2,4);
    Pz = T(3,4);

    % The configuration parameters determine what n1,n2,n4 values are used
    % and how many solutions are determined which have values of -1 or +1.
    
    % default configuration    
    isL=1;  % arm to Left
    isU=1;  % elbow Up
    isN=1;  % wrist Not flipped

    if isL
        n1 = -1; %#ok<UNRCH>
    else
        n1 = 1; %#ok<UNRCH>
    end
    if isU
        if n1 == 1 %#ok<UNRCH>
            n2 = 1;
        else
            n2 = -1;
        end
    else
        if n1 == 1 %#ok<UNRCH>
            n2 = -1;
        else
            n2 = 1;
        end
    end
    if isN
        n4 = 1; %#ok<UNRCH>
    else
        n4 = -1; %#ok<UNRCH>
    end


    %
    % Solve for theta(1)
    % 
    % r is defined in equation 38, p. 39.
    % theta(1) uses equations 40 and 41, p.39, 
    % based on the configuration parameter n1
    %

    r=sqrt(Px^2 + Py^2);
    if (n1 == 1)
        theta(1)= atan2(Py,Px) + asin(d3/r);
    else
        theta(1)= atan2(Py,Px) + pi - asin(d3/r);
    end

    %
    % Solve for theta(2)
    %
    % V114 is defined in equation 43, p.39.
    % r is defined in equation 47, p.39.
    % Psi is defined in equation 49, p.40.
    % theta(2) uses equations 50 and 51, p.40, based on the configuration 
    % parameter n2
    %

    V114= Px*cos(theta(1)) + Py*sin(theta(1));
    r=sqrt(V114^2 + Pz^2);
    Psi = acos((a2^2-d4^2-a3^2+V114^2+Pz^2)/(2.0*a2*r));
    if ~isreal(Psi)
        warning('RTB:ikine6s:notreachable', 'point not reachable');
        theta = [NaN; NaN; NaN; NaN; NaN; NaN];
        return
    end
    theta(2) = atan2(Pz,V114) + n2*Psi;

    %
    % Solve for theta(3)
    %
    % theta(3) uses equation 57, p. 40.
    %

    num = cos(theta(2))*V114+sin(theta(2))*Pz-a2;
    den = cos(theta(2))*Pz - sin(theta(2))*V114;
    theta(3) = atan2(a3,d4) - atan2(num, den);

    %
    % Solve for theta(4)
    %
    % V113 is defined in equation 62, p. 41.
    % V323 is defined in equation 62, p. 41.
    % V313 is defined in equation 62, p. 41.
    % theta(4) uses equation 61, p.40, based on the configuration 
    % parameter n4
    %

    V113 = cos(theta(1))*Ax + sin(theta(1))*Ay;
    V323 = cos(theta(1))*Ay - sin(theta(1))*Ax;
    V313 = cos(theta(2)+theta(3))*V113 + sin(theta(2)+theta(3))*Az;
    theta(4) = atan2((n4*V323),(n4*V313));
    %[(n4*V323),(n4*V313)]

    %
    % Solve for theta(5)
    %
    % num is defined in equation 65, p. 41.
    % den is defined in equation 65, p. 41.
    % theta(5) uses equation 66, p. 41.
    %
     
    num = -cos(theta(4))*V313 - V323*sin(theta(4));
    den = -V113*sin(theta(2)+theta(3)) + Az*cos(theta(2)+theta(3));
    theta(5) = atan2(num,den);
    %[num den]

    %
    % Solve for theta(6)
    %
    % V112 is defined in equation 69, p. 41.
    % V122 is defined in equation 69, p. 41.
    % V312 is defined in equation 69, p. 41.
    % V332 is defined in equation 69, p. 41.
    % V412 is defined in equation 69, p. 41.
    % V432 is defined in equation 69, p. 41.
    % num is defined in equation 68, p. 41.
    % den is defined in equation 68, p. 41.
    % theta(6) uses equation 70, p. 41.
    %

    V112 = cos(theta(1))*Ox + sin(theta(1))*Oy;
    V132 = sin(theta(1))*Ox - cos(theta(1))*Oy;
    V312 = V112*cos(theta(2)+theta(3)) + Oz*sin(theta(2)+theta(3));
    V332 = -V112*sin(theta(2)+theta(3)) + Oz*cos(theta(2)+theta(3));
    V412 = V312*cos(theta(4)) - V132*sin(theta(4));
    V432 = V312*sin(theta(4)) + V132*cos(theta(4));
    num = -V412*cos(theta(5)) - V332*sin(theta(5));
    den = - V432;
    theta(6) = atan2(num,den);