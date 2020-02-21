function res = ang_linspace(a,b,n)

if nargin<3
    n=100;
end

a = wrapToPi(a);
b = wrapToPi(b);

if abs(b-a)>pi
    if b>a
        res = linspace(a,b-2*pi,n);
    else
        res = linspace(a,b+2*pi,n);
    end
else
    res = linspace(a,b);
end
    