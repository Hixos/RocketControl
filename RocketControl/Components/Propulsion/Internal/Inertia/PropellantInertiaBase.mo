within RocketControl.Components.Propulsion.Internal.Inertia;

partial model PropellantInertiaBase
  parameter Modelica.Units.SI.Mass m_0 "Initial mass";
  parameter Modelica.Units.SI.Mass m_residual = m_0*0.01 "Residual mass at the end of combustion";
  Interfaces.MassPropertiesOutput massOut annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput mdot annotation(
    Placement(visible = true, transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-88, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
initial equation
  massOut.m = m_0;
equation
  if massOut.m > m_residual then
    der(massOut.m) = -mdot;
  else
    der(massOut.m) = 0;
  end if;
  annotation(
    Diagram,
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
end PropellantInertiaBase;
