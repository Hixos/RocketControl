within RocketControl.Interfaces.Debug;

model ComposeAeroState
AeroStateOutput out annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput alpha annotation(
    Placement(visible = true, transformation(origin = {-100, 90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput beta annotation(
    Placement(visible = true, transformation(origin = {-100, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput mach annotation(
    Placement(visible = true, transformation(origin = {-100, 22}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput altitude annotation(
    Placement(visible = true, transformation(origin = {-100, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v[3] annotation(
    Placement(visible = true, transformation(origin = {-100, -56}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput w[3] annotation(
    Placement(visible = true, transformation(origin = {-100, -90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  out.alpha = alpha;
  out.beta = beta;
  out.altitude = altitude;
  out.mach = mach;
  out.v = v;
  out.w = w;
  annotation(
    Icon(graphics = {Line(origin = {9.59965, 45.1944}, points = {{-89.5996, 44.8056}, {90.4004, -45.1944}, {-89.5996, 10.8056}}), Line(origin = {9.50306, 0}, points = {{-89.5031, 20}, {90.4969, 0}, {-89.5031, -20}}), Line(origin = {10, -28}, points = {{-90, -28}, {90, 28}}), Line(origin = {10, -45}, points = {{-90, -45}, {90, 45}}), Text(origin = {4, -125}, extent = {{-160, 25}, {160, -25}}, textString = "a,b,m,h,v,w")}));
end ComposeAeroState;
