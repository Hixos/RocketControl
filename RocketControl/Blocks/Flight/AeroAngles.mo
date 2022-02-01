within RocketControl.Blocks.Flight;

block AeroAngles
  extends Icon;
  Modelica.Blocks.Interfaces.RealInput v_body annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput track(displayUnit = "deg", quantity = "Angle", unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {110, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end AeroAngles;
