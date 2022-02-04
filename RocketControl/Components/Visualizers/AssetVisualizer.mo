within RocketControl.Components.Visualizers;

model AssetVisualizer
  RocketControl.Components.Parts.StaticFrame staticFrame annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape fixedShape(height = 0.15, length = 2, shapeType = "cone", width = 0.15) annotation(
    Placement(visible = true, transformation(origin = {30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Visualizers.FixedFrame fixedFrame annotation(
    Placement(visible = true, transformation(origin = {30, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(
    Placement(visible = true, transformation(origin = {-66, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Visualizers.SignalArrow signalArrow(color = {255, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {-26, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
 // signalArrow.r_head = aeroStateSensor.aeroStateOutput.v;
  connect(fixedFrame.frame_a, staticFrame.frame_b) annotation(
    Line(points = {{20, -32}, {0, -32}, {0, 0}, {-40, 0}}, color = {95, 95, 95}));
  connect(staticFrame.frame_b, fixedShape.frame_a) annotation(
    Line(points = {{-40, 0}, {-9.8, 0}, {-9.8, 30}, {20, 30}}));
  connect(frame_a, staticFrame.frame_a) annotation(
    Line(points = {{-102, 0}, {-60, 0}}));
 connect(absoluteVelocity.frame_a, frame_a) annotation(
    Line(points = {{-76, 52}, {-84, 52}, {-84, 0}, {-102, 0}}));
 connect(absoluteVelocity.v, signalArrow.r_head) annotation(
    Line(points = {{-54, 52}, {-26, 52}, {-26, 74}}, color = {0, 0, 127}, thickness = 0.5));
 connect(signalArrow.frame_a, staticFrame.frame_b) annotation(
    Line(points = {{-36, 86}, {-32, 86}, {-32, 0}, {-40, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Ellipse(lineColor = {179, 204, 208}, fillColor = {234, 234, 234}, fillPattern = FillPattern.Sphere, extent = {{-100, 100}, {100, -100}}), Polygon(origin = {-46, -40}, fillColor = {91, 91, 91}, fillPattern = FillPattern.Solid, points = {{-2, -12}, {8, -22}, {22, 12}, {12, 22}, {-22, 8}, {-2, -12}}), Line(origin = {25.5114, -29.1575}, rotation = -90, points = {{-18, -18}, {24, 24}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Line(origin = {-43.1155, -11.4371}, rotation = 180, points = {{-18, -18}, {36, -18}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Polygon(origin = {-15, -9}, fillColor = {155, 183, 193}, fillPattern = FillPattern.Solid, points = {{-43, -33}, {25, 35}, {45, 45}, {35, 25}, {-33, -43}, {-37, -39}, {-43, -33}}), Line(origin = {52.2533, 55.5403}, points = {{-18, -18}, {18, 18}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Text(origin = {-10, -254}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
end AssetVisualizer;
