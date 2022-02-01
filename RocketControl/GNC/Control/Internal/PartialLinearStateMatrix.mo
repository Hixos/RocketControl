within RocketControl.GNC.Control.Internal;

block PartialLinearStateMatrix
parameter Integer nx(min = 1) annotation(Evaluate = true);
parameter Integer nu(min = 1) annotation(Evaluate = true);

Modelica.Blocks.Interfaces.RealOutput B[nx,nu] annotation(
    Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Interfaces.RealOutput A[nx,nx] annotation(
    Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));


  annotation(
    Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {-4, -96},lineColor = {0, 0, 127}, extent = {{10, 10}, {90, 90}}, textString = "B"), Text(origin = {98, -2},lineColor = {0, 0, 127}, extent = {{-90, 10}, {-10, 90}}, textString = "A"), Line(points = {{-90, 0}, {90, 0}}, color = {192, 192, 192}), Line(points = {{0, -90}, {0, 90}}, color = {192, 192, 192}), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end PartialLinearStateMatrix;
