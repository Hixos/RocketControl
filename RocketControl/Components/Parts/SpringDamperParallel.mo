within RocketControl.Components.Parts;

model SpringDamperParallel "Linear spring and linear damper in parallel"
  extends RocketControl.Interfaces.PartialConditionalEnablePort;
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  import Modelica.Mechanics.MultiBody.Types;
  import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
  import Modelica.Mechanics.MultiBody.Frames.resolve2;
  parameter SI.TranslationalSpringConstant c(final min=0) "Spring constant";

  parameter SI.TranslationalDampingConstant d(final min=0) = 0
    "Damping constant";
  parameter SI.Position r_rel_start[3] = zeros(3);
  SI.Position r_rel[3](start = r_rel_start);
  SI.Length s;
protected
  SI.Force f_d[3] "Damping force";
   SI.Force f[3];
equation
  r_rel = resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  s = norm(r_rel);
  if enable then
    f_d = d*der(r_rel);
    f = c*r_rel + f_d;
  else
    f_d = zeros(3);
    f = zeros(3);
  end if;
  frame_a.f = -f;
  frame_b.f = resolveRelative(f, frame_a.R, frame_b.R);
  
  frame_a.t = zeros(3);
  frame_b.t = zeros(3);
  
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-52, -40}, {68, -40}}), Text(lineColor = {128, 128, 128}, extent = {{100, -25}, {136, -50}}, textString = "b"), Text(origin = {-98, -6},extent = {{-150, 105}, {150, 135}}, textString = "c=%c"), Line(points = {{38, -70}, {80, -70}}), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}), Line(points = {{-80, -70}, {-52, -70}}), Line(visible = false, points = {{-100, -101}, {-100, -80}, {-6, -80}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Line(points = {{-80, 40}, {-60, 40}, {-45, 10}, {-15, 70}, {15, 10}, {45, 70}, {60, 40}, {80, 40}}), Line(points = {{80, 40}, {80, -70}}), Rectangle(fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-52, -40}, {38, -100}}), Line(points = {{80, 0}, {100, 0}}), Text(origin = {-92, -4},extent = {{-150, 70}, {150, 100}}, textString = "d=%d"), Text(visible = false, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Line(points = {{-52, -100}, {68, -100}}), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}), Line(points = {{-80, 40}, {-80, -70}}), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}), Text(visible = false, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}), Line(points = {{-100, 0}, {-80, 0}})}));
end SpringDamperParallel;
