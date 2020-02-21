function res = ang_multi_linspace(a,b,n)

sz = size(a,1);
res = zeros(sz,n);

for k=1:sz
    res(k,:)=ang_linspace(a(k),b(k),n);
end