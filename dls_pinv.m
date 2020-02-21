%   DLS Damped Least-Squares Pseudoinverse.
%   The function is invoked as X = DLS(J,lambda),
%   where J is a (MxN) matrix and lambda is a (Mx1) vector.
%   DLS(J,lambda) produces a matrix X of the same dimensions as J'.
%   For lambda=0, DLS(J,lambda)=pinv(J), but if J is close to a singularity
%   and lambda is greater than 0, instabilities are avoided.
%   If no lambda is provided, a default value is used
%   The computation is based on SVD(A)

function X=dls_pinv(J,lambda)

[m,n] = size(J);

if n > m
   if nargin<2
      X = dls_pinv(J').';
   else
      X = dls_pinv(J',lambda).';
   end
else
   [U,S,V] = svd(J,0);
   s = diag(S);
   
   if nargin<2      
       maniplty=s(n);
       lambda=ones(n,1)*exp(-10*maniplty);
   end
   
   s = diag(s./(s.*s+lambda));
   X = V*s*U.';
end