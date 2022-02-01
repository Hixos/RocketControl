within RocketControl.Math.Quaternions;

function quatinv
  extends Modelica.Icons.Function;
  input Real[4] q;
  output Real[4] qinv;
algorithm
  qinv[1:3] := -q[1:3];
  qinv[4] := q[4];
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end quatinv;
