within RocketControl.GNC;

model FeedbackIntegrator
  extends RocketControl.Components.Interfaces.PartialConditionalEnablePort;
  parameter Integer n(min = 1) = 1;
  Modelica.Blocks.Interfaces.RealInput ref[n] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput err_int[n] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput feedback[n] annotation(
    Placement(visible = true, transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
equation
  if enable then
//err_int = zeros(n);
    der(err_int) = ref - feedback;
  else
    err_int = zeros(n);
  end if;
  annotation(
    Icon(graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Line(visible = false, points = {{50, 70}, {80, 70}}, color = {255, 0, 0}), Line(visible = false, points = {{-80, -70}, {-50, -70}}, color = {255, 0, 0}), Ellipse(origin = {-50, 0}, extent = {{-24, 24}, {24, -24}}), Text(origin = {40, -14}, extent = {{-26, 26}, {26, -26}}, textString = "s"), Rectangle(origin = {40, 0}, extent = {{-40, 40}, {40, -40}}), Text(origin = {40, 20}, extent = {{-26, 26}, {26, -26}}, textString = "1"), Line(origin = {39.72, -2.6}, points = {{-19.8536, 2.20711}, {20.1464, 2.20711}}), Line(origin = {-35.75, -18.42}, points = {{-19.8536, 2.20711}, {-5.8536, 2.20711}}), Line(origin = {-49.88, -2.21}, points = {{-19.8536, 2.20711}, {-5.8536, 2.20711}}), Line(origin = {-43.18, 4.73}, points = {{-19.8536, 2.20711}, {-19.8536, -11.7929}}), Line(origin = {-87, 0}, points = {{-13, 0}, {13, 0}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 6), Line(origin = {-25, -62}, points = {{25, -38}, {25, -18}, {-25, -18}, {-25, 38}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 6), Line(origin = {-12.5967, -0.168941}, points = {{-13, 0}, {13, 0}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 6), Line(origin = {90, 0}, points = {{-10, 0}, {10, 0}})}));
end FeedbackIntegrator;
