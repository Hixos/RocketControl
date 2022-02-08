within RocketControl.Math.Vectors;

function angle "Returns the angle between two vectors"
extends Modelica.Icons.Function;
input Real[3] v1;
input Real[3] v2;
input Real v_small = 1e-6;
output SI.Angle a;
algorithm
  if norm(v1) < v_small then
    a := 0;
    return;
  end if;
  
  if norm(v2) < v_small then
    a := 0;
    return;
  end if;
  
  a := acos(v1*v2/(norm(v1)*norm(v2)));
annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end angle;
