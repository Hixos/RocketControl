within RocketControl.Components.LaunchPad;

model LaunchRailPresenceSensor "Detects wheter frame_lug is within a distance of frame_rail along a certain direction"
  import Modelica.Mechanics.MultiBody.Frames.resolve2;
  parameter Real n[3] = {0, 0, 1};
  parameter SI.Length rail_length;
  SI.Position s[3];
  SI.Length d;
  Modelica.Blocks.Interfaces.BooleanOutput y(start = true) annotation(
    Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lug annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -38}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_rail annotation(
    Placement(visible = true, transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
//initial equation
//  y = true;
equation
  s = resolve2(frame_rail.R, frame_lug.r_0 - frame_rail.r_0);
  d = s * n;
  if d > rail_length then
  y = false;
  else
  y = true;
  end if;
//  when d > rail_length then
//    y = false;
//  end when;
  frame_rail.f = zeros(3);
  frame_rail.t = zeros(3);
  frame_lug.f = zeros(3);
  frame_lug.t = zeros(3);
  annotation(
    Icon(graphics = {Polygon(origin = {-9, 8}, fillColor = {222, 222, 222}, fillPattern = FillPattern.Forward, points = {{17, 88}, {-71, -78}, {-71, -88}, {71, -88}, {49, -80}, {-55, -80}, {17, 62}, {17, 88}}), Polygon(origin = {-2, 30}, fillColor = {85, 122, 162}, fillPattern = FillPattern.VerticalCylinder, points = {{-28, -42}, {-14, -50}, {-14, -42}, {26, 38}, {28, 50}, {20, 42}, {-20, -38}, {-28, -42}}), Line(origin = {-35, -41}, points = {{11, 21}, {-11, -23}}, thickness = 0.75), Line(origin = {-27.06, -24.0629}, points = {{-4.93552, 0.159125}, {3.06448, 4.15913}, {5.06448, -3.84087}, {5.06448, -3.84087}}, thickness = 0.75), Line(origin = {-43.4078, -60.2878}, points = {{-5, 3.87873}, {-3, -4.12127}, {5, -2.12127}, {5, -2.12127}}, thickness = 0.75), Text(origin = {-566.66, 40}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "lug"), Text(origin = {-620.66, -52}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "rail"), Text(origin = {0, 8}, lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name")}));
end LaunchRailPresenceSensor;
