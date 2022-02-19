within RocketControl.Components.Propulsion.Internal;

model PartialRocketMotor
  parameter SI.Time Isp = 180 "Specific impulse in seconds";
  parameter Modelica.Units.SI.Mass m_0 "Initial mass";
  parameter Modelica.Units.SI.Position r_cm[3] "Center of mass position";
  
  RocketControl.Components.Propulsion.Internal.Inertia.PartialPropellantInertia inertia(m_0 = m_0)  annotation(
    Placement(visible = true, transformation(origin = {-20, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  GenericThrust generic_thrust(Isp = Isp) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  RocketControl.Components.Parts.BodyVariableMass propellant(animation = false, r_CM = r_cm, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {40, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput m_dot annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  connect(generic_thrust.frame_b, frame_b) annotation(
    Line(points = {{0, 9}, {0, 98}}, color = {95, 95, 95}));
  connect(inertia.massOut, propellant.massInput) annotation(
    Line(points = {{-10, -50}, {23, -50}, {23, -54}, {34, -54}}));
  connect(propellant.frame_a, frame_b) annotation(
    Line(points = {{40, -40}, {40, 60}, {0, 60}, {0, 98}}));
  connect(m_dot, generic_thrust.m_dot) annotation(
    Line(points = {{-100, 0}, {-10, 0}}, color = {0, 0, 127}));
  connect(m_dot, inertia.mdot) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, -50}, {-31, -50}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0}), graphics = {Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "M2000R"), Rectangle(origin = {0.51, 47.73}, fillColor = {85, 85, 85}, fillPattern = FillPattern.VerticalCylinder, extent = {{-59.94, 48.27}, {59.94, -48.27}}), Polygon(origin = {1, -30}, fillColor = {43, 88, 85}, fillPattern = FillPattern.VerticalCylinder, points = {{-21, 30}, {-51, 10}, {-71, -30}, {71, -30}, {49, 10}, {19, 30}, {5, 30}, {-21, 30}}), Text(origin = {2, -176}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
end PartialRocketMotor;
