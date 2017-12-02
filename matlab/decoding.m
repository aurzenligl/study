function [mod] = decoding()
mod.gen = @gen;
end

function [data] = gen()
nSym = 24;

symbols = demodulate.symbols_gen(nSym);
softbits = demodulate.demod_qpsk(symbols);

scramSeq = normalize.scram_qpsk_gen(nSym);
softbitsDescr = normalize.descr_qpsk(softbits, scramSeq);

[invNoise, nInvNoise] = normalize.invnoise_gen(nSym);
softbitsNorm = normalize.norm(2, invNoise, nInvNoise, softbitsDescr);

symbolsMixed = flipud(reshape(symbols, 4, 6)'(:,[1 4 3 2]));

data.symbols0 = symbolsMixed(:,1);
data.symbols1 = symbolsMixed(:,2);
data.symbols2 = symbolsMixed(:,3);
data.symbols3 = symbolsMixed(:,4);
data.scramSeq = scramSeq;
data.invNoise0 = invNoise(1,:);
data.invNoise1 = invNoise(2,:);
data.invNoise2 = invNoise(3,:);
data.nInvNoise = nInvNoise;
data.softbitsRef = softbitsNorm;
end
