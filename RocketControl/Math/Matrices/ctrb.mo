within RocketControl.Math.Matrices;

function ctrb
extends Modelica.Icons.Function;
input Real A[:,size(A,1)];
input Real B[size(A,1),:];
output Real C[size(A,1), size(B,2)*size(A,1)];
protected
 Integer n;
 Integer m;
 Integer j;
 Real An[size(A,1), size(A,1)];
algorithm  
  n := size(A,1);
  m := size(B,2);
  An := identity(n);
  for i in 1:n-1 loop
    j := (i-1)*m + 1;
    C[:,j:j+m-1] := An*B;
    An := An*A;
  end for;
annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end ctrb;
