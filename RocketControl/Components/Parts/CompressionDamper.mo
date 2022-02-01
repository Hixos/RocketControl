within RocketControl.Components.Parts;

model CompressionDamper "Damper that acts only on a specific direction and can be enabled / disabled"
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  import Modelica.Mechanics.MultiBody.Frames.resolve2;
  import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
  parameter Real n[3] = {1, 0, 0} "Direction along which the damper is acting";
  parameter Modelica.Units.SI.DampingCoefficient d;
  SI.Position s(fixed = false, start = 0);
  SI.Position r_rel[3];
  SI.Force f[3];
equation
  r_rel = resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  s = r_rel * n;
  frame_b.t = zeros(3);
  frame_a.t = zeros(3);
  if der(s) < 0 and s < 0 then
    f = -d * der(r_rel);
  else
    f = zeros(3);
  end if;
  frame_a.f = f;
  frame_b.f = resolveRelative(-f, frame_a.R, frame_b.R);
  annotation(
    Icon(graphics = {Text(origin = {4, -176}, lineColor = {0, 0, 255}, extent = {{-150, 85}, {150, 45}}, textString = "%name"), Text(origin = {0, -12}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Rectangle(origin = {8, 0}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {30, -30}}), Line(origin = {7.40383, -1.76063e-06}, points = {{-101, 0}, {-60, 0}}), Line(origin = {7.40383, -1.76063e-06}, points = {{30, 0}, {100, 0}}), Line(origin = {7.40383, -1.76063e-06}, points = {{-60, -30}, {-60, 30}}), Text(origin = {0, 18}, extent = {{-150, -75}, {150, -45}}, textString = "d=%d"), Line(visible = false, origin = {7.40383, -1.76063e-06}, points = {{-100, -99}, {-100, -25}, {-10, -25}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Line(origin = {7.40383, -1.76063e-06}, points = {{-60, -30}, {60, -30}}), Line(origin = {7.40383, -1.76063e-06}, points = {{-60, 30}, {60, 30}}), Polygon(origin = {-30, 60}, rotation = 180, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}}), Line(origin = {-23.8859, 59.9583}, points = {{44, 0}, {4, 0}}, thickness = 0.75), Polygon(origin = {30, 60}, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}})}));
end CompressionDamper;
