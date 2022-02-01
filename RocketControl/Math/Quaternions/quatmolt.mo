within RocketControl.Math.Quaternions;

function quatmolt
  extends Modelica.Icons.Function;
  input Real[4] q1;
  input Real[4] q2;
  output Real[4] qm;
algorithm
  qm[1:3] := q2[4] * q1[1:3] + q1[4] * q2[1:3] - cross(q1[1:3], q2[1:3]);
  qm[4] := q1[4] * q2[4] - q1[1:3] * q2[1:3];
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end quatmolt;
