function res = multi_linspace(a,b,n)

if nargin<3
    n=100;
end

sz = size(a,1);
res = zeros(sz,n);

for k=1:sz
    res(k,:)=linspace(a(k),b(k),n);
end