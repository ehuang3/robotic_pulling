function [ a, b ] = dft( s, x, f, n )
%DFT Discrete Fourier Transform.
%   [A,B] = DFT(S,X,F,N) computes the coefficients of the fourier series 
%   S_N(X) for the (1/F)-periodic function S(X).

%% Compute discrete fourier transform.

% Assert that width of x matches the period.
assert(abs(abs(max(x)-min(x))-1/f) < 1e-7,'Period mismatch');

% Assert valid dimensions.
assert(size(s,1)==1,'Dimension mismatch');
assert(size(x,1)==1,'Dimension mismatch');
assert(size(s,2)==size(x,2),'Dimension mismatch');

% Compute coefficients.
x_min = min(x);
x_max = max(x);
a = zeros([1,n+1]);
b = zeros([1,n+1]);
for i = 0:n
%     a(i+1) = 2*f * trapz(x,s.*cos(2*pi*i*f.*x));
%     b(i+1) = 2*f * trapz(x,s.*sin(2*pi*i*f.*x));

    fun = @(z) linterp(x,s,mod(z+pi,2*pi)-pi) .* cos(2*pi*i*f.*z);
    a(i+1) = 2*f*integral(fun,x_min,x_max);

    fun = @(z) linterp(x,s,mod(z+pi,2*pi)-pi) .* sin(2*pi*i*f.*z);
    b(i+1) = 2*f*integral(fun,x_min,x_max);
end

end

