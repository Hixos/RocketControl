within RocketControl.GNC.Control;

model Control2Deflection
  final parameter Real M[4, 4] = [-0.5, 0, 0.5, 0; 0, 0.5, 0, -0.5; -0.25, -0.25, -0.25, -0.25; -0.25, 0.25, -0.25, 0.25] annotation(
    Evaluate = true);
  final parameter Real Minv[4, 4] = inv(M) annotation(
    Evaluate = true);
  Modelica.Blocks.Interfaces.RealInput u[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput deflection[4] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  deflection = Minv * cat(1, u, {0});
  annotation(
    Icon(graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "c2f")}));
end Control2Deflection;
