within RocketControl.Math.Matrices;

function obsv
extends Modelica.Icons.Function;
input Real A[:,size(A,1)];
input Real C[:,size(A,1)];
output Real O[size(C,1)*size(A,1), size(A,1)];
protected
 Integer n;
 Integer p;
 Integer j;
 Real An[size(A,1), size(A,1)];
algorithm  
  n := size(A,1);
  p := size(C,1);
  An := identity(n);
  for i in 1:n-1 loop
    j := (i-1)*p + 1;
    O[j:j+p-1,:] := C*An;
    An := An*A;
  end for;
annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end obsv;
