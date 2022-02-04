within RocketControl.GNC.Control;

model Control2Deflection
  final parameter Real M[4, 4] = [-0.5, 0, 0.5, 0; 0, 0.5, 0, -0.5; -0.25, -0.25, -0.25, -0.25; -0.25, 0.25, -0.25, 0.25] annotation(
    Evaluate = true);
  final parameter Real Minv[4, 4] = inv(M) annotation(
    Evaluate = true);
    
  parameter Boolean use_ds = false;
  Modelica.Blocks.Interfaces.RealInput u[4](each final unit = "rad", each final quantity="Angle", each displayUnit="deg") annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    
  Modelica.Blocks.Interfaces.RealOutput deflection[4](each final unit = "rad", each final quantity="Angle", each displayUnit="deg") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  if not use_ds then
   u[4] = 0;
   end if;
  deflection = Minv * u;
  annotation(
    Icon(graphics = {Rectangle(fillColor = {255, 231, 213}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {31, -78}, fillColor = {255, 224, 201}, extent = {{-65, 22}, {65, -22}}, textString = "d1,d2,d3,d4", horizontalAlignment = TextAlignment.Left), Line(origin = {-5, 0}, points = {{-75, 0}, {83, 0}}, thickness = 0.5, arrow = {Arrow.None, Arrow.Open}, arrowSize = 15), Text(origin = {-33, 78}, fillColor = {255, 224, 201}, extent = {{-65, 22}, {65, -22}}, textString = "dy,dp,dr,ds", horizontalAlignment = TextAlignment.Left), Text(origin = {0, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end Control2Deflection;
