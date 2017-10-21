function [] = does_this_name_matter(shift)
fs = 1000;
fswave = 8;
t = -0.1:1/fs:0.1;
n = length(t);

f = cos((t)*2*pi*fswave);
g = square((t+shift)*2*pi*fswave, 25);

t2 = -0.3:1/fs:0.3;

for i = 1:n-1
    h(i) = sum(f(1:i).*g(n+1-i:n));
end
h(n) = sum(f.*g);
for i = 1:n-1
    h(n+i) = sum(f(i+1:n).*g(1:n-i));
end

t2 = -(length(t)-1):length(t)-1;

subplot(5,1,1)
plot(t, f, '.')
subplot(5,1,2)
plot(t, g, '.')
subplot(5,1,3)
plot(t2, h/length(t), '.')
subplot(5,1,4)
plot(t2, xcorr(f,g)/length(t), '.')
subplot(5,1,5)
plot(t2, (xcorr(f,g)-h)/length(t), '.')
end