function [mod] = decoding()
#mod.symbols_gen = @symbols_gen;
#mod.demod_qpsk = @demod_qpsk;
mod.gen = @gen;
end

function [data] = gen()
nSym = 24;

symbols = demodulate.symbols_gen(nSym);
softbits = demodulate.demod_qpsk(symbols);

[invNoise, nInvNoise] = normalize.invnoise_gen(nSym);
softbitsNorm = normalize.norm(2, invNoise, nInvNoise, softbits);

# split symbols to 4 anri symbols
data.symbols = symbols;
data.invNoise0 = invNoise(1,:);
data.invNoise1 = invNoise(2,:);
data.invNoise2 = invNoise(3,:);
data.nInvNoise = nInvNoise;
data.softbitsRef = softbitsNorm;
end
