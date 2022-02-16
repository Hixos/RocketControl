within RocketControl.Rockets.Lynx;

model LynxParachute2
  extends Rockets.Internal.PartialRocket;
  RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation drogue_position(animation = false, r = {0.5, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-14, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LinearAerodynamicsWithCanards.LinearAerodynamics linearAerodynamics annotation(
    Placement(visible = true, transformation(origin = {-8, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
    Placement(visible = true, transformation(origin = {-76, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_r = true)  annotation(
    Placement(visible = true, transformation(origin = {40, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation main_position(animation = false, r = {0.3, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-14, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation line_attachment_point(animation = false, r = {0.8, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-14, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.Parachute2 drogue(Cd = 1, c = 10000, c_stowed = 2000, d = 3, d_stowed = 30, initial_surface = 0.02, line_length = 3, mass = 0.2, opening_transient_duration = 0.2, stow_radius = 0.075, surface = 0.4) annotation(
    Placement(visible = true, transformation(origin = {44, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.Parachute2 main(Cd = 1, c = 10000, c_stowed = 10000, d = 3, d_stowed = 100, initial_surface = 0.1, line_length = 2, mass = 1, opening_transient_duration = 0.4, stow_radius = 0.075, surface = 4) annotation(
    Placement(visible = true, transformation(origin = {44, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit1(r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_r = true) annotation(
    Placement(visible = true, transformation(origin = {40, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
  connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
    Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
  connect(lynxBody.ref_center, ref_center) annotation(
    Line(points = {{-40, 0}, {100, 0}}));
  connect(drogue_position.frame_a, lynxBody.ref_center) annotation(
    Line(points = {{-24, 24}, {-40, 24}, {-40, 0}}, color = {95, 95, 95}));
  connect(linearAerodynamics.frame_b, lynxBody.ref_center) annotation(
    Line(points = {{-18, 88}, {-40, 88}, {-40, 0}}, color = {95, 95, 95}));
  connect(vectorConstant.v, linearAerodynamics.finDeflection) annotation(
    Line(points = {{-64, 84}, {-18, 84}, {-18, 82}}, color = {0, 0, 127}, thickness = 0.5));
  connect(drogue_position.frame_b, freeMotionScalarInit.frame_a) annotation(
    Line(points = {{-4, 24}, {8, 24}, {8, -46}, {30, -46}}, color = {95, 95, 95}));
  connect(lynxBody.ref_center, main_position.frame_a) annotation(
    Line(points = {{-40, 0}, {-40, -32}, {-24, -32}}, color = {95, 95, 95}));
  connect(lynxBody.ref_center, line_attachment_point.frame_a) annotation(
    Line(points = {{-40, 0}, {-40, 44}, {-24, 44}}));
  connect(freeMotionScalarInit.frame_b, drogue.chute) annotation(
    Line(points = {{50, -46}, {74, -46}, {74, 50}, {54, 50}}));
  connect(bus.drogue_deploy, drogue.enable) annotation(
    Line(points = {{100, 90}, {44, 90}, {44, 59}}, color = {255, 0, 255}));
  connect(drogue_position.frame_b, drogue.stowed_position) annotation(
    Line(points = {{-4, 24}, {10, 24}, {10, 43}, {34, 43}}));
  connect(line_attachment_point.frame_b, drogue.chute_link) annotation(
    Line(points = {{-4, 44}, {13, 44}, {13, 50}, {34, 50}}, color = {95, 95, 95}));
  connect(main_position.frame_b, main.stowed_position) annotation(
    Line(points = {{-4, -32}, {34, -32}, {34, 6}}, color = {95, 95, 95}));
  connect(line_attachment_point.frame_b, main.chute_link) annotation(
    Line(points = {{-4, 44}, {18, 44}, {18, 12}, {34, 12}}, color = {95, 95, 95}));
  connect(main_position.frame_b, freeMotionScalarInit1.frame_a) annotation(
    Line(points = {{-4, -32}, {-2, -32}, {-2, -74}, {30, -74}}, color = {95, 95, 95}));
  connect(freeMotionScalarInit1.frame_b, main.chute) annotation(
    Line(points = {{50, -74}, {72, -74}, {72, 12}, {54, 12}}, color = {95, 95, 95}));
  connect(bus.main_deploy, main.enable) annotation(
    Line(points = {{100, 90}, {44, 90}, {44, 22}}, color = {255, 0, 255}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
end LynxParachute2;
