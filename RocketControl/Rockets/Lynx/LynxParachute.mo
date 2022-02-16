within RocketControl.Rockets.Lynx;

model LynxParachute
  extends Rockets.Internal.PartialRocket;
  RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation drogue_position(animation = false, r = {0.5, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-6, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Parts.Parachute drogue(Cd = 1, c = 10000, c_stowed = 100000, d_stowed = 30000, initial_surface = 0.02, line_length = 3, mass = 0.3, opening_transient_duration = 0.3, surface = 0.4, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {56, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.Parachute main(Cd = 0.9, c = 10000, c_stowed = 100000, d_stowed = 30000, initial_surface = 0.1, line_length = 2, mass = 1, opening_transient_duration = 0.5, surface = 4, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {56, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation main_position(animation = false, r = {0.3, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-6, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Aerodynamics.Aerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {-8, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
  connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
    Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
  connect(lynxBody.ref_center, ref_center) annotation(
    Line(points = {{-40, 0}, {100, 0}}));
  connect(drogue_position.frame_a, lynxBody.ref_center) annotation(
    Line(points = {{-16, 50}, {-40, 50}, {-40, 0}}, color = {95, 95, 95}));
  connect(drogue_position.frame_b, drogue.frame_a) annotation(
    Line(points = {{4, 50}, {30, 50}, {30, 52}, {46, 52}}));
  connect(main_position.frame_b, main.frame_a) annotation(
    Line(points = {{4, 20}, {30, 20}, {30, 22}, {46, 22}}));
  connect(main_position.frame_a, lynxBody.ref_center) annotation(
    Line(points = {{-16, 20}, {-40, 20}, {-40, 0}}, color = {95, 95, 95}));
  connect(bus.drogue_deploy, drogue.enable) annotation(
    Line(points = {{100, 90}, {56, 90}, {56, 62}}, color = {255, 0, 255}));
  connect(bus.main_deploy, main.enable) annotation(
    Line(points = {{100, 90}, {56, 90}, {56, 32}}, color = {255, 0, 255}));
  connect(aerodynamics.frame_b, lynxBody.ref_center) annotation(
    Line(points = {{-18, 88}, {-40, 88}, {-40, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
end LynxParachute;
