t=0:0.012:10;

s=zeros(length(t),1);
sdot=zeros(length(t),1);

L=norm([0;0.45;1.515]-[0.45;0;1.515]);

vc_max=0.3;
ac_max=0.1;

for k=1:length(t)
    [sk,sdotk]=bang_bang(L,t(k),vc_max,ac_max);
    s(k)=sk;
    sdot(k)=sdotk;
end

figure;
plot(s);
figure;
plot(sdot);