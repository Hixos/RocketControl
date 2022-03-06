within RocketControl.GNC.Control.LinearOutputMatrices;

model AngularRatesOutput
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-110, -10}, {-90, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput C[3,9] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  C = [0,0,0,1,0,0,0,0,0;
       0,0,0,0,1,0,0,0,0;
       0,0,0,0,0,1,0,0,0];
annotation(
    Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Line(points = {{0, -90}, {0, 90}}, color = {192, 192, 192}), Text(origin = {104, -48}, lineColor = {0, 0, 127}, extent = {{-90, 10}, {-10, 90}}, textString = "C"), Line(points = {{-90, 0}, {90, 0}}, color = {192, 192, 192})}));
end AngularRatesOutput;
