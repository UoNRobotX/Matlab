function [y,Py,ysigma,Pxy] = Unscented(x,Px,func,ny,xsigma)
% This function performs a basic Unscented Tranform of y = f(x)

%--------------------------------------------------------------------------
% Unscented Parameters

alpha = 1; beta = 2; kappa = 0;
n = size(x,1); len = 2*n + 1;
lambda = alpha^2*(n + kappa) - n; gamma = sqrt(n + lambda);

%--------------------------------------------------------------------------
% Weight Calculation of Means and Covariances

Wp = repmat(1/(2*(n+lambda)),1,2*n);
Wm = [Wp, lambda/(n+lambda)];
Wc = [Wp, lambda/(n+lambda)+(1-alpha^2+beta)]; 

%--------------------------------------------------------------------------
% Input Sigma Point Generation

if isempty(xsigma)
    xmat = repmat(x,1,n);
    xsigma = zeros(n,len);
    Stranc = gamma*chol(Px)';
    xsigma(:,1:n) = xmat + Stranc;
    xsigma(:,n+1:2*n) = xmat - Stranc;
    xsigma(:,end) = x;
end

%--------------------------------------------------------------------------
% Transformation of Sigma Points through f(x)

ysigma = zeros(ny,len);
for count = 1:len
    [ysigma(:,count),~] = func(xsigma(:,count));
end

%--------------------------------------------------------------------------
% Obtain Transformed Mean, Covariance and Cross-Covariance

y = sum(repmat(Wm,ny,1).*ysigma,2);
Py = zeros(ny,ny); Pxy = zeros(n,ny);
ymat = ysigma - repmat(y,1,len); xmat = xsigma - repmat(x,1,len);
for count = 1:len
   Py = Py + Wc(count).*(ymat(:,count)*ymat(:,count)');
   Pxy = Pxy + Wc(count).*(xmat(:,count)*ymat(:,count)');    
end

end