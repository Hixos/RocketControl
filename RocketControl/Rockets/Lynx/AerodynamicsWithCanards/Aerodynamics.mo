within RocketControl.Rockets.Lynx.AerodynamicsWithCanards;

model Aerodynamics
  extends RocketControl.Aerodynamics.PartialAerodynamics(redeclare AerodynamicForce aerodynamicForce);
  Modelica.Blocks.Interfaces.RealInput finDeflection[4] annotation(
    Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(finDeflection, aerodynamicForce.finDeflection) annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Aerodynamics;
