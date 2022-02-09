within RocketControl.Components.Parts;

model ParachuteLine "Spring that reacts only when compressed"
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  extends RocketControl.Interfaces.PartialConditionalEnablePort;
  import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
  import Modelica.Mechanics.MultiBody.Frames.resolve2;
  parameter SI.Length extended_length;
  parameter Modelica.Units.SI.ModulusOfElasticity c;
  parameter Modelica.Units.SI.ModulusOfElasticity c_stowed;
  parameter Modelica.Units.SI.DampingCoefficient d_stowed;
  parameter SI.Position r_rel_start[3] = zeros(3);
  SI.Position r_rel[3](start = r_rel_start);
  SI.Length s;
  SI.Length s_def;
  SI.Force f[3];
  SI.Force tension;
  Modelica.Blocks.Interfaces.BooleanOutput extended annotation(
    Placement(visible = true, transformation(origin = {106, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
initial equation
extended = false;
equation
  r_rel = resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  s = norm(r_rel);
  
  when s >= extended_length then
    extended = true;
  end when;
  if enable and s >= extended_length then
    f = -c*(norm(r_rel) - extended_length)*r_rel/norm(r_rel);
    tension = norm(f);
    s_def = s - extended_length;
  elseif not enable then
    f = -c_stowed*r_rel - d_stowed*der(r_rel);
    tension = 0;
    s_def = 0;
  else  
    f = zeros(3);
    tension = 0;    
    s_def = 0;
  end if;
  
  frame_a.f = f;
  frame_b.f = resolveRelative(-f, frame_a.R, frame_b.R);
  
  frame_a.t = zeros(3);
  frame_b.t = zeros(3);
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Rectangle(fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-52, -40}, {38, -100}}), Line(points = {{-52, -100}, {68, -100}}), Line(points = {{-80, 40}, {-60, 40}, {-45, 10}, {-15, 70}, {15, 10}, {45, 70}, {60, 40}, {80, 40}}), Text(origin = {-82, 154}, extent = {{-150, -75}, {150, -45}}, textString = "c=%c"), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Text(origin = {-85.999, 138}, lineColor = {128, 128, 128}, extent = {{99.9988, -29}, {135.999, -58}}, textString = "e"), Line(points = {{-52, -40}, {68, -40}}), Line(points = {{-80, -70}, {-52, -70}}), Line(points = {{38, -70}, {80, -70}}), Text(origin = {-82, 122}, extent = {{-150, -75}, {150, -45}}, textString = "d=%d"), Line(visible = false, points = {{-100, -101}, {-100, -80}, {-6, -80}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Line(origin = {-80, -16}, points = {{0, 56}, {0, -54}}), Line(origin = {80, -15}, points = {{0, 55}, {0, -55}}), Line(origin = {-87, 0}, points = {{-7, 0}, {7, 0}}), Line(origin = {88, 0}, points = {{-8, 0}, {8, 0}})}));
end ParachuteLine;
