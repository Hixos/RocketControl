within RocketControl.Blocks.Trasformations;

model body2ned
  Modelica.Blocks.Interfaces.RealInput x_b[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q[4] annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput x_w[3] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  x_w = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve1(q, x_b);
  annotation(
    Icon(graphics = {Ellipse(fillColor = {152, 222, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, endAngle = 360), Text(origin = {-130, 20}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "x_b"), Text(origin = {-130, -100}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "q_bw"), Text(origin = {110, -20}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "x_w"), Line(origin = {-90, -60}, points = {{-10, 0}, {10, 0}, {10, 0}}), Line(origin = {-90, 60}, points = {{-10, 0}, {10, 0}}), Line(origin = {-42.3245, 48.3402}, points = {{19.6465, 30}, {-10.3536, -7.10543e-15}, {11.6465, -40}}, thickness = 0.75, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 5), Line(origin = {-41.6744, 45.3402}, points = {{-11, 3}, {25, -5}}, thickness = 0.75, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 5), Line(origin = {31.0955, -41.7198}, points = {{-20.1708, -34.1708}, {-20.1708, 9.82918}, {19.8292, 29.8292}}, thickness = 0.75, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 5), Line(origin = {28.9255, -40.8897}, points = {{-18, 9}, {18, -9}}, thickness = 0.75, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 5), Line(origin = {14.14, 23.92}, points = {{-19.0121, 25.0121}, {-3.01206, 23.0121}, {6.98794, 17.0121}, {14.9879, 1.01206}, {18.9879, -24.9879}}, color = {255, 0, 0}, thickness = 0.75, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Text(origin = {0, -131}, lineColor = {0, 0, 255}, extent = {{-180, 29}, {180, -29}}, textString = "%name")}));
end body2ned;
