within RocketControl.Blocks.Math.Vector;

block VectorSine
  parameter Real[3] A;
  parameter SI.Frequency f;
  parameter SI.Angle[3] phase(each displayUnit = "deg") = {0, 0, 0};
  parameter Real[3] b = {0, 0, 0};
  extends Internal.VectorIcon;
  Modelica.Blocks.Interfaces.RealOutput y[3] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine(amplitude = A[1], f = f, offset = b[1], phase = phase[1]) annotation(
    Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine1(amplitude = A[2], f = f, offset = b[2], phase = phase[2]) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine2(amplitude = A[3], f = f, offset = b[3], phase = phase[3]) annotation(
    Placement(visible = true, transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(sine.y, y[1]) annotation(
    Line(points = {{12, 50}, {60, 50}, {60, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(sine1.y, y[2]) annotation(
    Line(points = {{12, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(sine2.y, y[3]) annotation(
    Line(points = {{12, -50}, {60, -50}, {60, 0}, {110, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Line(origin = {-37.87, 0.22}, points = {{-49.9838, 21.8869}, {-43.9838, 1.88686}, {-33.9838, -18.1131}, {-19.9838, -30.1131}, {-5.98375, -34.1131}, {8.0162, -30.1131}, {20.0162, -18.1131}, {32.0162, 1.8869}, {38.0162, 21.8869}}, color = {255, 0, 0}, thickness = 0.5), Line(origin = {38.28, 44.34}, rotation = 180, points = {{-49.9838, 21.8869}, {-43.9838, 1.88686}, {-33.9838, -18.1131}, {-19.9838, -30.1131}, {-5.98375, -34.1131}, {8.0162, -30.1131}, {20.0162, -18.1131}, {32.0162, 1.8869}, {38.0162, 21.8869}}, color = {255, 0, 0}, thickness = 0.5), Line(origin = {1.95, -40.82}, points = {{-87, 0}, {87, 0}}, arrow = {Arrow.Open, Arrow.Open}, arrowSize = 7), Line(origin = {1.64, -17.2}, points = {{-87, 0}, {87, 0}}, color = {134, 134, 134}, arrowSize = 7), Line(origin = {0, 2.89}, points = {{0, -20}, {0, 20}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 5), Text(origin = {25, 4}, extent = {{-23, 18}, {23, -18}}, textString = "b"), Text(origin = {-1, -62}, extent = {{-23, 18}, {23, -18}}, textString = "2 pi f")}));
end VectorSine;
