function ret = uni_square(T,DUTY)
  if nargin == 1
    DUTY = 50
  end
  if isOctave()
    ret = square(T, DUTY/100);
  else
    ret = square(T, DUTY);
  end
end