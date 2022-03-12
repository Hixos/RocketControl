within RocketControl.Blocks.Math.Vectors;

model Angle
 extends RocketControl.Icons.VectorBlock;
  Modelica.Blocks.Interfaces.RealOutput angle(unit="rad", quantity="Angle", displayUnit="deg") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v2[3] annotation(
    Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v1[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
if noEvent(norm(v1) > 1e-6) and noEvent(norm(v2) > 1e-6) then
 angle = acos(v1*v2/(norm(v1)*norm(v2)));
 else
 angle = 0;
 end if;
annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Angle;
