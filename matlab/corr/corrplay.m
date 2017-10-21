function [] = what_a_language(shiftstep)
shift = 0;
while true,
    shift = shift + shiftstep;
    corr(shift);
    pause(1e-20)
end
end