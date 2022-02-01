within RocketControl.Components.Sensors.TrueSensors;

model TrueAsset
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  Modelica.Blocks.Interfaces.RealOutput q[4] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  frame_a.f = {0, 0, 0};
  frame_a.t = {0, 0, 0};
  q = Modelica.Mechanics.MultiBody.Frames.Quaternions.from_T(frame_a.R.T);
  annotation(
    Icon(graphics = {Text(origin = {-10, -212}, lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "q"), Line(origin = {85, 0}, points = {{-15, 0}, {15, 0}})}));
end TrueAsset;
