within RocketControl.Interfaces.Debug;

model ComposeMassProperties
  RocketControl.Interfaces.MassPropertiesOutput out annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput m annotation(
    Placement(visible = true, transformation(origin = {-104, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-96, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput I[6] annotation(
    Placement(visible = true, transformation(origin = {-104, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-92, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  out.m = m;
  out.I[1, 1] = I[1];
  out.I[2, 2] = I[2];
  out.I[3, 3] = I[3];
  out.I[1, 2] = I[4];
  out.I[1, 3] = I[5];
  out.I[2, 3] = I[6];
  out.I[2, 1] = I[4];
  out.I[3, 1] = I[5];
  out.I[3, 2] = I[6];
  annotation(
    Icon(graphics = {Line(origin = {-52.397, 0.412098}, points = {{-48, 60}, {48, 60}, {48, -60}, {-40, -60}, {40, -60}}), Line(origin = {49, 0}, points = {{53, 0}, {-53, 0}}), Text(origin = {-68, 196.667}, extent = {{28, -126.667}, {-28, -88.6667}}, textString = "m"), Text(origin = {-68, 70.67}, extent = {{28, -126.67}, {-28, -88.67}}, textString = "I")}));
end ComposeMassProperties;
