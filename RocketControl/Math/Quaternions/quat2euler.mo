within RocketControl.Math.Quaternions;

function quat2euler "Converts a quaternion to the equivalent ZYX euler angle representation"
  extends Modelica.Icons.Function;
  input Real q_in[4];
  output SI.Angle eul[3];
protected
  Real q[4];
algorithm
  q := q_in / norm(q_in);
  eul[1] := atan2(2 * q[4] * q[3] + 2 * q[1] * q[2], q[4] ^ 2 + q[1] ^ 2 - q[2] ^ 2 - q[3] ^ 2);
  eul[2] := asin(2 * q[4] * q[2] - 2 * q[1] * q[3]);
  eul[3] := atan2(2 * q[4] * q[1] + 2 * q[2] * q[3], q[4] ^ 2 - q[1] ^ 2 - q[2] ^ 2 + q[3] ^ 2);
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end quat2euler;
