within RocketControl.Components.Parts;

model CompressionSpring "Spring that reacts only when compressed"
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  import Modelica.Mechanics.MultiBody.Frames.resolve2;
  import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
  parameter Real n[3] = {1, 0, 0} "Direction along which the spring is acting";
  parameter Modelica.Units.SI.ModulusOfElasticity c;
  SI.Position s(fixed = false, start = 0);
  SI.Position r_rel[3];
  SI.Force f[3];
equation
  r_rel = resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  s = r_rel * n;
  frame_b.t = zeros(3);
  frame_a.t = zeros(3);
  if s < 0 then
    f = -c * r_rel;
  else
    f = zeros(3);
  end if;
  frame_a.f = f;
  frame_b.f = resolveRelative(-f, frame_a.R, frame_b.R);
  annotation(
    Icon(graphics = {Line(points = {{-100, 0}, {-58, 0}, {-43, -30}, {-13, 30}, {17, -30}, {47, 30}, {62, 0}, {100, 0}}), Line(origin = {-23.8859, 59.9583}, points = {{44, 0}, {4, 0}}, thickness = 0.75), Text(origin = {0, 22}, extent = {{-150, -75}, {150, -45}}, textString = "c=%c"), Text(origin = {0, -12}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Text(origin = {4, -176}, lineColor = {0, 0, 255}, extent = {{-150, 85}, {150, 45}}, textString = "%name"), Polygon(origin = {30, 60}, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}}), Polygon(origin = {-30, 60}, rotation = 180, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}})}));
end CompressionSpring;
