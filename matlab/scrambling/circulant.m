function [y] = circulant(x)
n = length(x);
k = kron(1:n, ones(n,1));
y = x(1 + mod(k' - k, n));
end
