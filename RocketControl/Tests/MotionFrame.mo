within RocketControl.Tests;

model MotionFrame
  import Modelica.Units.SI;
  import Modelica.Mechanics.MultiBody.Frames;
  parameter SI.Velocity p0[3] = {0, 0, 0};
  parameter SI.Velocity v0[3];
  parameter SI.Angle angles0[3](each displayUnit = "deg") = {0, 0, 0};
  final parameter Frames.Orientation R0 = Modelica.Mechanics.MultiBody.Frames.axesRotations({3, 2, 1}, angles0, {0, 0, 0});
  parameter SI.AngularVelocity w[3](each displayUnit = "deg/s") = {0, 0, 0};
  SI.Angle angles[3];
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {98, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {98, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
initial equation
  frame_b.r_0 = p0;
  angles = {0, 0, 0};
equation
  Connections.root(frame_b.R);
  der(frame_b.r_0) = v0;
  der(angles) = w;
  frame_b.R = Frames.absoluteRotation(Frames.axesRotations({1, 2, 3}, angles, der(angles)), R0);
  annotation(
    Icon(graphics = {Ellipse(origin = {0.04, 1.95}, fillColor = {255, 255, 255}, fillPattern = FillPattern.CrossDiag, extent = {{-99.96, 99.95}, {99.96, -99.95}}, endAngle = 360)}));
end MotionFrame;
