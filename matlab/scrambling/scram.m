function [self] = scram
self.testSeq = @testSeq;
self.makeSeq = @makeSeq;
self.seq = @seq;
self.seqDensity = @seqDensity;
self.seqXors = @seqXors;
self.seqXorsNeg = @seqXorsNeg;
end

% 1. see how it behaves with taps [1,2,3,4], write jumps down
% 2. try to calculate performance of scrambling function
% 3. how does gmpy work, is it helpful?
% 4. understand white-papers
% 5. read both jump implementations (pusch + npusch)

function [x] = seq(n)
x = eye(31);
for i = 1:n
    x = [x(:,2:31), mod(sum(x(:,[1,2,3,4]), 2), 2)];
end
end

function [q] = seqDensity(n)
x = eye(31);
for i = 1:n
    x = [x(:,2:31), mod(sum(x(:,[1,2,3,4]), 2), 2)];
    q(i) = sum(sum(x));
end
end

function [q] = seqXors(n)
x = eye(31);
for i = 1:n
    x = [x(:,2:31), mod(sum(x(:,[1,2,3,4]), 2), 2)];
    q(i) = sum(logical(sum(circulant_shift(x))));
end
end

%{
possible jumps for taps [1, 4]
-------------

65536: (2**16)
    65492
    65534
    65535 (-1)

32768: (2**15)
    32752 (-16)

16384: (2**14)
    16376 (-8)

8192: (2**13)
    8188 (-4)

4096: (2**12)
    4092
    4094 (-2)

2048: (2**11)
    2043
    2044
    2046 (-2)

1024: (2**10)
    1023 (-1)

512: (2**9)
    496 (-16)

256: (2**8)
    248 (-8)

128: (2**7)
    124 (-4)

64: (2**6)
    62 (-2)

32: (2**5)
    31 (-1)
%}

function [q] = seqXorsNeg(n)
x = eye(31);
for i = 1:n
    x = [x(:,2:31), mod(sum(x(:,[1,4]), 2), 2)];
    q(i) = sum(logical(sum(!circulant_shift(x))));
end
end

function [seq] = makeSeq(nSeq, cinit)
% Nc and initial condition of polynomial x1 as defined in 36.211 7.2
Nc = 1600;
x1 = [1, zeros(1, 30)];
x2 = de2bi(cinit, 31);
seq = zeros(1, nSeq);

for n = 1 : (Nc + nSeq)
    x1(n+31) = mod(sum(x1([0 3] + n)), 2);
    x2(n+31) = mod(sum(x2([0 1 2 3] + n)), 2);
end

for n = 1 : nSeq
    seq(n) = mod(x1(Nc + n) + x2(Nc + n), 2);
end
end

function [data] = testSeq(nSeq, cinit)
seq = makeSeq(nSeq, cinit);

data.cinit = int32(cinit);
data.nSeq = int32(nSeq);
data.seqRef = uint8(seq);
end
