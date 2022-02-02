within RocketControl.Components.Propulsion.Internal;

model SolidMotorAndTank
  parameter SI.Time Isp = 180 "Specific impulse in seconds";
  parameter Modelica.Units.SI.Length Dext "External diameter";
  parameter Modelica.Units.SI.Length Din_0 "Initial internal diameter";
  parameter Modelica.Units.SI.Length h "Height";
  parameter Modelica.Units.SI.Mass m_0 "Initial mass";
  RocketControl.Components.Propulsion.Internal.Inertia.SolidPropellantInertia solidPropellantInertia(Dext = Dext, Din_0 = Din_0, h = h, m_0 = m_0) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Parts.BodyVariableMass bodyVariableMass(animation = false,r_CM = {h / 2, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {40, 4}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput thrust annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Components.Propulsion.Internal.GenericMotor genericMotor(Isp = Isp) annotation(
    Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(solidPropellantInertia.massOut, bodyVariableMass.massInput) annotation(
    Line(points = {{10, 0}, {34, 0}}));
  connect(bodyVariableMass.frame_a, frame_b) annotation(
    Line(points = {{40, 14}, {40, 60}, {0, 60}, {0, 98}}));
  connect(solidPropellantInertia.mdot, genericMotor.mdot) annotation(
    Line(points = {{-8, 0}, {-30, 0}}, color = {0, 0, 127}));
  connect(genericMotor.thrust, thrust) annotation(
    Line(points = {{-50, 0}, {-100, 0}}, color = {0, 0, 127}));
  connect(genericMotor.frame_b, frame_b) annotation(
    Line(points = {{-40, 10}, {-40, 60}, {0, 60}, {0, 98}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "M2000R"), Text(origin = {2, -176}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Polygon(origin = {1, -30}, fillColor = {43, 88, 85}, fillPattern = FillPattern.VerticalCylinder, points = {{-21, 30}, {-51, 10}, {-71, -30}, {71, -30}, {49, 10}, {19, 30}, {5, 30}, {-21, 30}}), Rectangle(origin = {0.51, 47.73}, fillColor = {85, 85, 85}, fillPattern = FillPattern.VerticalCylinder, extent = {{-59.94, 48.27}, {59.94, -48.27}})}));
end SolidMotorAndTank;
