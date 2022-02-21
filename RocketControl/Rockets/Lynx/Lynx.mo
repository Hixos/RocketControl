within RocketControl.Rockets.Lynx;

model Lynx
 extends Rockets.Internal.PartialRocket;
  
    parameter SI.Duration start_delay = 0.5;
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation drogue_flange(animation = false, r = {0.5, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {16, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  replaceable RocketControl.Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {50, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation line_attachment_point(animation = false, r = {0.61, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.Parachute2 drogue(Cd = 0.9, I_x = 0.000241539, I_y = 0.002009527, c_flange = 2000, c_line = 10000, d_flange = 30, d_line = 3, flange_radius = 0.075, initial_surface = 0.02, line_length = 3, mass = 0.1, opening_transient_duration = 0.2, surface = 0.6) annotation(
    Placement(visible = true, transformation(origin = {70, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    
  RocketControl.Components.Parts.Parachute2 main(Cd = 0.7, I_x = 0.003381552, I_y = 0.028133387, c_flange = 10000, c_line = 10000, d_flange = 100, d_line = 3, flange_radius = 0.075, initial_surface = 0.1, line_length = 2, mass = 1.4, opening_transient_duration = 0.4, surface = 10.6) annotation(
    Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition drogue_pos annotation(
    Placement(visible = true, transformation(origin = {20, -20}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation main_flange(animation = false, r = {0.41, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {20, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(animation = false, r = {-0.408, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Components.Propulsion.M2000R m2000r(start_delay = start_delay) annotation(
    Placement(visible = true, transformation(origin = {-40, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(animation = false, r = {-0.009, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Body fuselage(I_11 = 0.053814847, I_21 = 0, I_22 = 3.268741422, I_31 = 0, I_32 = 0, I_33 = 3.268741422, animation = false, enforceStates = false, m = 14.124, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzle_trans(animation = false, r = {-1.150, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-40, -48}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  RocketControl.Components.Parts.SeparableFlange noseconeFlange(c = 100000, d = 1000,separation_duration = 0.3, separation_force = 500)  annotation(
    Placement(visible = true, transformation(origin = {-50, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Body nosecone(I_11 = 0.013576801, I_21 = 0, I_22 = 0.076867583, I_31 = 0, I_32 = 0, I_33 = 0.076867583, animation = false, enforceStates = false, m = 5.848, r_CM = {0.091, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fuselage_trans(animation = false, r = {-0.4, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nose_flange_trans(animation = false, r = {0.743, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit1(angle_1(fixed = true), angle_2(fixed = true), angle_3(fixed = true), r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_angle = true, use_r = true) annotation(
    Placement(visible = true, transformation(origin = {-27, 55}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition nosecone_pos annotation(
    Placement(visible = true, transformation(origin = {-12, 56}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition fuselage_pos annotation(
    Placement(visible = true, transformation(origin = {8, 56}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition propellant_pos annotation(
    Placement(visible = true, transformation(origin = {-52, -54}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition main_pos annotation(
    Placement(visible = true, transformation(origin = {20, -84}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    
    SI.Mass total_mass;
    SI.Position center_of_mass[3];
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
 RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
    Placement(visible = true, transformation(origin = {58, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
equation
  total_mass = nosecone.m + fuselage.m + main.mass + drogue.mass + m2000r.m2000r.propellant.m;
  center_of_mass = (nosecone.m * nosecone_pos.r_rel + fuselage.m * fuselage_pos.r_rel + drogue.mass * drogue_pos.r_rel + main.mass * main_pos.r_rel + m2000r.m2000r.propellant.m * propellant_pos.r_rel) / total_mass;
  connect(lug_bow.frame_b, frame_lug_bow) annotation(
    Line(points = {{-80, -10}, {-100, -10}, {-100, 60}}, color = {95, 95, 95}));
  connect(lug_aft.frame_a, fuselage_trans.frame_a) annotation(
    Line(points = {{-60, -30}, {-10, -30}, {-10, 0}, {0, 0}, {0, 10}}));
  connect(fuselage.frame_a, fuselage_trans.frame_b) annotation(
    Line(points = {{0, 80}, {0, 30}}, color = {95, 95, 95}));
  connect(nosecone.frame_a, noseconeFlange.frame_b) annotation(
    Line(points = {{-50, 80}, {-50, 66}}));
  connect(noseconeFlange.frame_a, nose_flange_trans.frame_b) annotation(
    Line(points = {{-50, 46}, {-50, 30}}, color = {95, 95, 95}));
  connect(main_flange.frame_b, main.flange_position) annotation(
    Line(points = {{30, -100}, {50, -100}, {50, -66}, {60, -66}}));
  connect(line_attachment_point.frame_b, drogue.chute_link) annotation(
    Line(points = {{30, -70}, {48, -70}, {48, -44}, {60, -44}}, color = {95, 95, 95}));
  connect(line_attachment_point.frame_b, main.chute_link) annotation(
    Line(points = {{30, -70}, {45, -70}, {45, -74}, {60, -74}}, color = {95, 95, 95}));
 connect(drogue_flange.frame_b, drogue.flange_position) annotation(
    Line(points = {{26, -36}, {60, -36}}, color = {95, 95, 95}));
  connect(ref_center, lug_aft.frame_a) annotation(
    Line(points = {{100, 0}, {-10, 0}, {-10, -30}, {-60, -30}}));
  connect(drogue_pos.frame_b, drogue.chute) annotation(
    Line(points = {{24, -20}, {86, -20}, {86, -40}, {80, -40}}, color = {95, 95, 95}));
  connect(lug_aft.frame_b, frame_lug_aft) annotation(
    Line(points = {{-80, -30}, {-100, -30}, {-100, -40}}, color = {95, 95, 95}));
  connect(lug_aft.frame_a, lug_bow.frame_a) annotation(
    Line(points = {{-60, -30}, {-50, -30}, {-50, -10}, {-60, -10}}, color = {95, 95, 95}));
  connect(lug_aft.frame_a, nose_flange_trans.frame_a) annotation(
    Line(points = {{-60, -30}, {-50, -30}, {-50, 10}}));
 connect(lug_aft.frame_a, nozzle_trans.frame_a) annotation(
    Line(points = {{-60, -30}, {-40, -30}, {-40, -38}}, color = {95, 95, 95}));
 connect(drogue_flange.frame_a, lug_aft.frame_a) annotation(
    Line(points = {{6, -36}, {-20, -36}, {-20, -30}, {-60, -30}}, color = {95, 95, 95}));
  connect(line_attachment_point.frame_a, lug_aft.frame_a) annotation(
    Line(points = {{10, -70}, {-20, -70}, {-20, -30}, {-60, -30}}, color = {95, 95, 95}));
  connect(main_flange.frame_a, lug_aft.frame_a) annotation(
    Line(points = {{10, -100}, {-20, -100}, {-20, -30}, {-60, -30}}, color = {95, 95, 95}));
 connect(nozzle_trans.frame_b, m2000r.frame_b) annotation(
    Line(points = {{-40, -58}, {-40, -70}}));
  connect(aerodynamics.frame_b, ref_center) annotation(
    Line(points = {{40, 80}, {28, 80}, {28, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(nose_flange_trans.frame_b, freeMotionScalarInit1.frame_a) annotation(
    Line(points = {{-50, 30}, {-50, 40}, {-27, 40}, {-27, 50}}));
  connect(freeMotionScalarInit1.frame_b, nosecone.frame_a) annotation(
    Line(points = {{-27, 60}, {-27, 72}, {-50, 72}, {-50, 80}}));
  connect(nosecone_pos.frame_b, nosecone.frame_a) annotation(
    Line(points = {{-12, 60}, {-12, 72}, {-50, 72}, {-50, 80}}));
  connect(nosecone_pos.frame_a, nose_flange_trans.frame_a) annotation(
    Line(points = {{-12, 52}, {-12, 43}, {-20, 43}, {-20, 10}, {-50, 10}}, color = {95, 95, 95}));
  connect(fuselage.frame_a, fuselage_pos.frame_b) annotation(
    Line(points = {{0, 80}, {0, 62}, {8, 62}, {8, 60}}, color = {95, 95, 95}));
  connect(fuselage_pos.frame_a, fuselage_trans.frame_a) annotation(
    Line(points = {{8, 52}, {8, 10}, {0, 10}}, color = {95, 95, 95}));
 connect(nozzle_trans.frame_a, propellant_pos.frame_a) annotation(
    Line(points = {{-40, -38}, {-52, -38}, {-52, -50}}, color = {95, 95, 95}));
  connect(propellant_pos.frame_b, m2000r.frame_b) annotation(
    Line(points = {{-52, -58}, {-52, -70}, {-40, -70}}));
 connect(drogue_pos.frame_a, drogue_flange.frame_a) annotation(
    Line(points = {{16, -20}, {6, -20}, {6, -36}}));
  connect(main_pos.frame_b, main.chute) annotation(
    Line(points = {{24, -84}, {84, -84}, {84, -70}, {80, -70}}, color = {95, 95, 95}));
  connect(main_pos.frame_a, main_flange.frame_a) annotation(
    Line(points = {{16, -84}, {10, -84}, {10, -100}}));
  connect(bus.main_deploy, main.enable) annotation(
    Line(points = {{100, 100}, {100, 10}, {70, 10}, {70, -60}}, color = {255, 0, 255}));
  connect(bus.drogue_deploy, drogue.enable) annotation(
    Line(points = {{100, 100}, {100, 10}, {70, 10}, {70, -30}}, color = {255, 0, 255}));
  connect(bus.drogue_deploy, noseconeFlange.enable) annotation(
    Line(points = {{100, 100}, {-80, 100}, {-80, 56}, {-59, 56}}, color = {255, 0, 255}));
 connect(vectorConstant.v, aerodynamics.finDeflection) annotation(
    Line(points = {{47, 42}, {32, 42}, {32, 74}, {40, 74}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
  Diagram(graphics = {Text(origin = {-31, -21}, extent = {{-19, 7}, {19, -7}}, textString = "ref_center", textStyle = {TextStyle.Italic})}));
end Lynx;
