function [mod] = normalize()
mod.norm = @norm;
mod.invnoise_gen = @invnoise_gen;
mod.norm_gen = @norm_gen;
mod.gen = @gen;
mod.scram_qpsk_gen = @scram_qpsk_gen;
mod.descr_qpsk = @descr_qpsk;
end

function [] = gen()
_store(norm_gen(2), 'Qpsk');
_store(norm_gen(4), '16Qam');
_store(norm_gen(6), '64Qam');
end

function [invNoise, nInvNoise] = invnoise_gen(nSymbols)
nAnriSyms = 4;
nScs = nSymbols / nAnriSyms;

invNoise = [0.1, 10, 0.0004, 4000; 1./rand(2, nAnriSyms)];
nInvNoise = diff([0; sort(randint(2, 1, nScs)); nScs]);
end

function [normSoftbits] = norm(sbPerSym, invNoise, nInvNoise, softbits)
scaleSyms = invNoise(repelems([1:3], [1:3; nInvNoise']), :)'(:);
scaleSbs = repelems(scaleSyms, [1:length(scaleSyms); sbPerSym * ones(1,length(scaleSyms))])';
normSoftbits = softbits .* scaleSbs;
end

function [data] = norm_gen(sbPerSym)
# consts
nSbs = 432;
nSyms = nSbs / sbPerSym;

# input
[invNoise, nInvNoise] = invnoise_gen(nSyms);
softbits = rand(nSbs,1)*2-1;

# output
softbitsRef = norm(sbPerSym, invNoise, nInvNoise, softbits);

data.invNoise0 = invNoise(1,:);
data.invNoise1 = invNoise(2,:);
data.invNoise2 = invNoise(3,:);
data.nInvNoise = nInvNoise;
data.softbits = softbits;
data.softbitsRef = softbitsRef;
end

function [scramseq] = scram_qpsk_gen(nSym)
bits = uint8(rand(nSym, 2));
scramseq = bits(:,1) * 128 + bits(:,2) * 64;
end

function [softbits] = descr_qpsk(softbits, scramseq)
bits = dec2bin(scramseq)(:,1:2)'(:) == '1';
reverser = bits * -2 + 1;
softbits = softbits .* reverser;
end

function [] = _store(data, suffix)
name = strcat('Normalize', suffix, '.mat');
save('-binary', name, 'data')
end
