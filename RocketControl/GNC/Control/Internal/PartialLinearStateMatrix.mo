within RocketControl.GNC.Control.Internal;

block PartialLinearStateMatrix
Modelica.Blocks.Interfaces.RealOutput B annotation(
    Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Interfaces.RealOutput A annotation(
    Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation

  annotation(
    Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {0, 4}, extent = {{-80, 68}, {80, -68}}, textString = "A,B"), Text(origin = {105, 21}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {105, -79}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end PartialLinearStateMatrix;
