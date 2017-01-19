function [ s ] = idft( w, a, b, f )
%IDFT Inverse discrete fourier transform.
%   S = IDFT(W,A,B) computes the inverse discrete fourier transform for 
%   the angles W given the fourier coefficients A and B.

%% Compute inverse discrete fourier transform.

% Check input dimensions.
assert(size(w,1)==1,'Dimension mismatch');
assert(size(a,1)==1,'Dimension mismatch');
assert(size(b,1)==1,'Dimension mismatch');
assert(size(a,2)==size(b,2),'Dimension mismatch');

% Compute inverse discrete fourier transform.
n = length(a)-1;
m = length(w);
X = repmat(w,[n,1]);
N = repmat((1:n)',[1,m]);
A = repmat(a(2:end)',[1,m]);
B = repmat(b(2:end)',[1,m]);
s = a(1)/2 + sum(A.*cos(2*pi*f.*N.*X) + B.*sin(2*pi*f.*N.*X));

end