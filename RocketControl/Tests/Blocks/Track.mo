within RocketControl.Tests.Blocks;

model Track
  RocketControl.Components.Blocks.Track track annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(
    Placement(visible = true, transformation(origin = {-50, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp ramp(duration = 10, height = 2 * pi) annotation(
    Placement(visible = true, transformation(origin = {-90, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sin sin annotation(
    Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Cos cos annotation(
    Placement(visible = true, transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
    Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine(amplitude = 300, f = 2, offset = 300) annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product1 annotation(
    Placement(visible = true, transformation(origin = {-10, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.UnitConversions.To_deg track_true annotation(
    Placement(visible = true, transformation(origin = {72, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.WrapAngle wrapAngle annotation(
    Placement(visible = true, transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ramp.y, sin.u) annotation(
    Line(points = {{-78, 12}, {-70, 12}, {-70, -10}, {-62, -10}}, color = {0, 0, 127}));
  connect(ramp.y, cos.u) annotation(
    Line(points = {{-78, 12}, {-70, 12}, {-70, 30}, {-62, 30}}, color = {0, 0, 127}));
  connect(sin.y, product1.u2) annotation(
    Line(points = {{-38, -10}, {-22, -10}, {-22, -16}}, color = {0, 0, 127}));
  connect(cos.y, product.u2) annotation(
    Line(points = {{-38, 30}, {-22, 30}, {-22, 24}}, color = {0, 0, 127}));
  connect(product1.u1, product.u1) annotation(
    Line(points = {{-22, -4}, {-22, 36}}, color = {0, 255, 255}));
  connect(product.y, track.v[1]) annotation(
    Line(points = {{2, 30}, {38, 30}, {38, 0}}, color = {0, 0, 127}));
  connect(product1.y, track.v[2]) annotation(
    Line(points = {{2, -10}, {38, -10}, {38, 0}}, color = {0, 0, 127}));
  connect(const.y, track.v[3]) annotation(
    Line(points = {{-38, -42}, {38, -42}, {38, 0}}, color = {0, 0, 127}));
  connect(ramp.y, wrapAngle.u) annotation(
    Line(points = {{-78, 12}, {-68, 12}, {-68, 50}, {18, 50}}, color = {0, 0, 127}));
  connect(wrapAngle.y, track_true.u) annotation(
    Line(points = {{42, 50}, {60, 50}}, color = {0, 0, 127}));
  connect(sine.y, product.u1) annotation(
    Line(points = {{-78, 90}, {-22, 90}, {-22, 36}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Track;
