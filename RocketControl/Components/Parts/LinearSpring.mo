within RocketControl.Components.Parts;

model LinearSpring "Spring that acts only on a specific direction and can be enabled / disabled"
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  extends Interfaces.PartialConditionalEnablePort;
  import Modelica.Mechanics.MultiBody.Frames.resolve2;
  import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
  parameter Real n[3] = {1, 0, 0} "Direction along which the spring is acting";
  parameter SI.Position s_0 = 0 "Undeformed spring length";
  parameter Modelica.Units.SI.ModulusOfElasticity c;
  SI.Position s(fixed = false, start = 0);
  SI.Force f[3];
equation
  s = resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0) * n;
  frame_b.t = zeros(3);
  frame_a.t = zeros(3);
  if enable then
    f = -c * (s - s_0) * n;
  else
    f = zeros(3);
  end if;
  frame_a.f = f;
  frame_b.f = resolveRelative(-f, frame_a.R, frame_b.R);
  annotation(
    Icon(graphics = {Line(points = {{-100, 0}, {-58, 0}, {-43, -30}, {-13, 30}, {17, -30}, {47, 30}, {62, 0}, {100, 0}}), Line(origin = {7.54808, 60}, points = {{-68, 0}, {40, 0}, {52, 0}}, thickness = 0.75), Line(origin = {49.9866, 59.6635}, points = {{-9.64645, 20}, {10.3536, -7.10543e-15}, {-9.64645, -20}}, thickness = 0.75), Text(origin = {0, 22}, extent = {{-150, -75}, {150, -45}}, textString = "c=%c"), Text(origin = {0, -12}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Text(origin = {4, -176}, lineColor = {0, 0, 255}, extent = {{-150, 85}, {150, 45}}, textString = "%name"), Line(origin = {-49.65, 60}, points = {{9.64645, 20}, {-10.3536, -7.10543e-15}, {9.64645, -20}}, thickness = 0.75)}));
end LinearSpring;
