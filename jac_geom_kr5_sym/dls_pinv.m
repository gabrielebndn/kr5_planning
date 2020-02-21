function dls=dls_pinv(X)

cond_max=1000;

[U,S,V]=svd(X);

sig_min=S(end,end);
sig_max=S(1,1);
sig_min2=sig_min*sig_min;
sig_max2=sig_max*sig_max;

if sig_max2/sig_min2>cond_max
    damping=(cond_max*sig_min2-sig_max2)/(1-cond_max);
else
    damping=0;
end

Sp=zeros(size(S,2),size(S,1));

for k=1:min(size(S,1),size(S,2))
    Sp(k,k)=S(k,k)/(S(k,k)*S(k,k)+damping);
end

dls=V*Sp*U.';



% cond_max=1000;
% 
% [U,S,V]=svd(X);
% 
% sig_max=S(1,1);
% sig_max2=sig_max*sig_max;
% 
% Sp=zeros(size(S,2),size(S,1));
% 
% for k=1:min(size(S,1),size(S,2))
%     sig=S(k,k);
%     sig2=sig*sig;
%     if sig_max2/sig2>cond_max
%         damping=(cond_max*sig2-sig_max2)/(1-cond_max);
%         Sp(k,k)=sig/(sig2+damping);
%     else
%         Sp(k,k)=1/sig;
%     end
% end
% 
% dls=V*Sp*U.';



% cond_max=1000;
% 
% [U,S,V]=svd(X);
% 
% sig_min=S(end,end);
% sig_max=S(1,1);
% 
% if sig_max/sig_min>cond_max
%     damping=(cond_max*sig_min-sig_max)/(1-cond_max);
% else
%     damping=0;
% end
% 
% Sp=zeros(size(S,2),size(S,1));
% 
% for k=1:min(size(S,1),size(S,2))
%     Sp(k,k)=S(k,k)/(S(k,k)*S(k,k)+damping*damping);
% end
% 
% dls=V*Sp*U.';



% cond_max=1000;
% 
% [U,S,V]=svd(X);
% 
% sig_max=S(1,1);
% 
% Sp=zeros(size(S,2),size(S,1));
% 
% for k=1:min(size(S,1),size(S,2))
%     sig=S(k,k);
%     if sig_max/sig>cond_max
%         damping=(cond_max*sig-sig_max)/(1-cond_max);
%         Sp(k,k)=sig/(sig*sig+damping*damping);
%     else
%         Sp(k,k)=1/sig;
%     end
% end
% 
% dls=V*Sp*U.';