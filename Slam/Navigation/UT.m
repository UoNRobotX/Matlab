function [y,Py,ysigma,Pxy] = UT(x,Px,func,ny,constraint,xsigma)

%
% Constrained Unscented Transform of y = f(x)
% using 2*n + 1 sigma points
%

n = length(x);              % Length of input vector
nsigma = 2*n+1;             % Number of sigma points

ysigma = zeros(ny,nsigma);   % Output sigma points

% Unscented transform parameters
alpha = 1;                  % 1e-4 to 1
kappa = 0;                  % 0 for state estimation, 3-n for parameter estimation
lambda = alpha^2*(n + kappa) - n;
gamma = sqrt(n + lambda);
beta = 2;                   % 2 for Gaussian distribution

% % Holmes et al. suggest the following for SLAM
% alpha = 1;                  % 1e-4 to 1
% kappa = 3 - n;              % 0 for state estimation, 3-n for parameter estimation
% lambda = alpha^2*(n + kappa) - n;
% gamma = sqrt(n + lambda);
% beta = 0;                   % 2 for Gaussian distribution


% Mean weights
Wm = [repmat(1/(2*(n+lambda)),1,2*n), lambda/(n+lambda)];

% Covariance weights
Wc = [repmat(1/(2*(n+lambda)),1,2*n), lambda/(n+lambda) + (1-alpha^2+beta)];


if nargin >= 6
    % Use externally provided sigma points
else
    % Generate sigma points
    xsigma = zeros(n,nsigma);   % Input sigma points
    S = chol(Px);
    for i = 1:n
        xsigma(:,i) = x + gamma*S(i,:).';
        xsigma(:,i+n) = x - gamma*S(i,:).';
    end
    xsigma(:,2*n+1) = x;
    
    % Apply constraints
    if nargin >= 5
        for i = 1:nsigma
            xsigma(:,i) = constraint(xsigma(:,i));
        end
    end
end

% Transform the sigma points through the function
for i = 1:nsigma
    ysigma(:,i) = func(xsigma(:,i));
end

% Unscented mean
y = zeros(ny,1);
for i = 1:nsigma
    y = y + Wm(i)*ysigma(:,i);
end

% Unscented covariance
Py = zeros(ny);
for i = 1:nsigma
    Py = Py + Wc(i)*((ysigma(:,i) - y)*(ysigma(:,i) - y).');
end

% Unscented cross covariance (optional)
if nargout >= 4
    Pxy = zeros(n,ny);
    for i = 1:2*n+1
        Pxy = Pxy + Wc(i)*((xsigma(:,i) - x)*(ysigma(:,i) - y).');
    end
end