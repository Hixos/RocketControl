within RocketControl.Blocks.Flight;

model Downrange
  extends Icon;
  Modelica.Blocks.Interfaces.RealInput x[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput d_downrange annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  d_downrange = sqrt(x[1] ^ 2 + x[2] ^ 2);
  annotation(
    Icon(graphics = {Ellipse(origin = {58, 56}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Line(origin = {-2.48, -6.47}, points = {{-46, -45}, {50, 53}}, color = {170, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {-2.86, -6.47}, points = {{-46, 17}, {-46, -45}, {14, -45}}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 10), Text(origin = {-130, -40}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "pos")}));
end Downrange;
