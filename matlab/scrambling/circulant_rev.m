function [y] = circulant_rev(x)
n = length(x);
k = kron(1:n, ones(n,1));
y = mod(rot90(k' - k + n - 1), n) + 1;
end
