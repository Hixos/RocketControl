within RocketControl.Blocks.Math.Matrices;

model DiagonalMatrix
  extends RocketControl.Icons.MatrixBlock;
  Modelica.Blocks.Interfaces.RealOutput diag[size(vec,1), size(vec,1)] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput vec[:] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0)));
equation
  diag = diagonal(vec);
  annotation(
    Icon(graphics = {Text(origin = {0, 10}, extent = {{-86, 78}, {86, -78}}, textString = "[d]"), Text(origin = {-4, -10}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}, coordinateSystem(grid = {2, 0})));
end DiagonalMatrix;
