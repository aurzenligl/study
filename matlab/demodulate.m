function [mod] = demodulate()
mod.symbols_gen = @symbols_gen;
mod.demod_qpsk = @demod_qpsk;
end

function [symbols] = symbols_gen(n)
symbols = (rand(n,1)*2-1) + j*(rand(n,1)*2-1);
end

function [softbits] = demod_qpsk(symbols)
softbits = [real(symbols), imag(symbols)]'(:);
end
