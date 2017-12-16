function [y] = circulant_shift(x)
n = length(x);
idx = circulant_rev(1:n) .+ ((1:n)' - 1) * n;
y = x(idx);
end
