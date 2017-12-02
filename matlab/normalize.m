function [mod] = normalize()
mod.norm = @norm;
mod.gen = @gen;
end

function [] = gen()
store(norm(2), 'Qpsk');
store(norm(4), '16Qam');
store(norm(6), '64Qam');
end

function [data] = norm(sbPerSym)
# consts
nAnriSyms = 4;
nSbs = 432;
nSyms = nSbs / sbPerSym;
nScs = nSyms / nAnriSyms;

# input
invNoise = [0.1,10,0.0004,4000; 1./rand(2,4)];
nInvNoise = diff([0; sort(randint(2, 1, nScs)); nScs]);
softbits = rand(nSbs,1)*2-1;

# output
scaleSyms = invNoise(repelems([1:3], [1:3; nInvNoise']), :)'(:);
scaleSbs = repelems(scaleSyms, [1:length(scaleSyms); sbPerSym * ones(1,length(scaleSyms))])';
softbitsRef = softbits .* scaleSbs;

data.invNoise = invNoise;
data.nInvNoise = nInvNoise;
data.softbits = softbits;
data.softbitsRef = softbitsRef;
end

function [] = store(data, suffix)
name = strcat('Normalize', suffix, '.mat');
save('-binary', name, 'data')
end
