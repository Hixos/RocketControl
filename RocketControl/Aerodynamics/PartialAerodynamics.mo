within RocketControl.Aerodynamics;

model PartialAerodynamics
  extends Internal.Icons.AerodynamicsIcon;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  replaceable PartialAerodynamicForce aerodynamicForce annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.AeroStateSensor aeroStateSensor annotation(
    Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(aerodynamicForce.frame_b, frame_b) annotation(
    Line(points = {{60, 0}, {80, 0}, {80, -40}, {-74, -40}, {-74, 0}, {-100, 0}}));
  connect(frame_b, aeroStateSensor.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}}));
  connect(aeroStateSensor.aeroStateOutput, aerodynamicForce.aeroState) annotation(
    Line(points = {{-20, 0}, {40, 0}}));
  annotation(
    Icon(graphics = {Text(origin = {10, -170}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
end PartialAerodynamics;
