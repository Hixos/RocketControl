within RocketControl.Aerodynamics;

function coefficientStrings
  output String array[Coefficients];
algorithm
  for c in Coefficients loop
    array[c] := String(c);
  end for;
end coefficientStrings;
