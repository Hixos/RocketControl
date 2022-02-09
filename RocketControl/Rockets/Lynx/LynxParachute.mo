within RocketControl.Rockets.Lynx;

model LynxParachute
  extends Rockets.Internal.PartialRocket;
  RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.Aerodynamics.Aerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {20, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = {1, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-16, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.Parachute drogue(Cd = 1, c = 10000, c_stowed = 100000, d_stowed = 6000, initial_surface = 0.02, line_length = 4, mass = 0.2, opening_transient_duration = 0.3, surface = 0.3, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.Parachute main(Cd = 0.9, c = 10000, c_stowed = 100000, d_stowed = 6000, initial_surface = 0.1, line_length = 2, mass = 1, opening_transient_duration = 0.4, surface = 5, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {50, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
  connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
    Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
  connect(lynxBody.ref_center, ref_center) annotation(
    Line(points = {{-40, 0}, {100, 0}}));
  connect(aerodynamics.frame_b, lynxBody.ref_center) annotation(
    Line(points = {{10, 78}, {-40, 78}, {-40, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_a, lynxBody.ref_center) annotation(
    Line(points = {{-26, 46}, {-40, 46}, {-40, 0}}, color = {95, 95, 95}));
  connect(bus.drogue_deploy, drogue.enable) annotation(
    Line(points = {{100, 90}, {50, 90}, {50, 59}}, color = {255, 0, 255}));
  connect(fixedTranslation.frame_b, main.frame_a) annotation(
    Line(points = {{-6, 46}, {14, 46}, {14, 16}, {40, 16}}));
  connect(bus.main_deploy, main.enable) annotation(
    Line(points = {{100, 90}, {64, 90}, {64, 30}, {50, 30}, {50, 26}}, color = {255, 0, 255}));
  connect(drogue.frame_a, fixedTranslation.frame_b) annotation(
    Line(points = {{40, 50}, {12, 50}, {12, 46}, {-6, 46}}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
end LynxParachute;
