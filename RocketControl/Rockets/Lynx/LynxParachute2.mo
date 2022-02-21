within RocketControl.Rockets.Lynx;

model LynxParachute2
  extends Rockets.Internal.PartialRocket;
  RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation drogue_flange(animation = false, r = {0.5, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LinearAerodynamicsWithCanards.LinearAerodynamics linearAerodynamics annotation(
    Placement(visible = true, transformation(origin = {-8, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
    Placement(visible = true, transformation(origin = {-76, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation main_flange(animation = false, r = {0.3, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-10, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation line_attachment_point(animation = false, r = {0.8, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.Parachute2 drogue(Cd = 1, c_line = 10000, c_flange = 2000, d_line = 3, d_flange = 30, initial_surface = 0.02, line_length = 3, mass = 0.2, opening_transient_duration = 0.2, flange_radius = 0.075, surface = 0.4) annotation(
    Placement(visible = true, transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    
  RocketControl.Components.Parts.Parachute2 main(Cd = 1, c_line = 10000, c_flange = 10000, d_line = 3, d_flange = 100, initial_surface = 0.1, line_length = 2, mass = 1, opening_transient_duration = 0.4, flange_radius = 0.075, surface = 4) annotation(
    Placement(visible = true, transformation(origin = {50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition relativePosition annotation(
    Placement(visible = true, transformation(origin = {66, -86}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition relativePosition1 annotation(
    Placement(visible = true, transformation(origin = {66, 30}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
equation
  connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
  connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
    Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
  connect(lynxBody.ref_center, ref_center) annotation(
    Line(points = {{-40, 0}, {100, 0}}));
  connect(drogue_flange.frame_a, lynxBody.ref_center) annotation(
    Line(points = {{-20, 30}, {-40, 30}, {-40, 0}}, color = {95, 95, 95}));
  connect(linearAerodynamics.frame_b, lynxBody.ref_center) annotation(
    Line(points = {{-18, 88}, {-40, 88}, {-40, 0}}, color = {95, 95, 95}));
  connect(vectorConstant.v, linearAerodynamics.finDeflection) annotation(
    Line(points = {{-64, 84}, {-18, 84}, {-18, 82}}, color = {0, 0, 127}, thickness = 0.5));
  connect(lynxBody.ref_center, main_flange.frame_a) annotation(
    Line(points = {{-40, 0}, {-40, -80}, {-20, -80}}, color = {95, 95, 95}));
  connect(lynxBody.ref_center, line_attachment_point.frame_a) annotation(
    Line(points = {{-40, 0}, {-31, 0}, {-31, -20}, {-20, -20}}));
  connect(bus.drogue_deploy, drogue.enable) annotation(
    Line(points = {{100, 90}, {50, 90}, {50, 59}}, color = {255, 0, 255}));
  connect(main_flange.frame_b, main.flange_position) annotation(
    Line(points = {{0, -80}, {40, -80}, {40, -77}}, color = {95, 95, 95}));
  connect(bus.main_deploy, main.enable) annotation(
    Line(points = {{100, 90}, {50, 90}, {50, -61}}, color = {255, 0, 255}));
  connect(line_attachment_point.frame_b, main.chute_link) annotation(
    Line(points = {{0, -20}, {16, -20}, {16, -70}, {40, -70}}, color = {95, 95, 95}));
  connect(drogue_flange.frame_b, drogue.flange_position) annotation(
    Line(points = {{0, 30}, {22, 30}, {22, 43}, {40, 43}}, color = {95, 95, 95}));
  connect(main_flange.frame_b, relativePosition.frame_a) annotation(
    Line(points = {{0, -80}, {26, -80}, {26, -86}, {60, -86}}, color = {95, 95, 95}));
  connect(relativePosition1.frame_a, drogue.flange_position) annotation(
    Line(points = {{60, 30}, {22, 30}, {22, 43}, {40, 43}}));
  connect(relativePosition1.frame_b, drogue.chute) annotation(
    Line(points = {{72, 30}, {76, 30}, {76, 50}, {60, 50}}, color = {95, 95, 95}));
  connect(main.chute, relativePosition.frame_b) annotation(
    Line(points = {{60, -70}, {74, -70}, {74, -86}, {72, -86}}, color = {95, 95, 95}));
  connect(line_attachment_point.frame_b, drogue.chute_link) annotation(
    Line(points = {{0, -20}, {16, -20}, {16, 50}, {40, 50}}, color = {95, 95, 95}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
end LynxParachute2;
