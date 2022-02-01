within RocketControl.Blocks.Math.Vector;

model UnitVector
  extends RocketControl.Icons.VectorBlock;
  parameter Integer n(min = 1) = 3 annotation(
    Evaluate = true);
  parameter Real v_small = 1e-6;
  Modelica.Blocks.Interfaces.RealOutput vu[n] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v[n] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Real vnorm;
equation
  vnorm = norm(v);
  if noEvent(vnorm < v_small) then
    vu = {1, 0, 0};
  else
    vu = v / vnorm;
  end if;
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(extent = {{-100, 100}, {100, -100}}, textString = "1")}));
end UnitVector;
