within RocketControl.Rockets.Lynx;

model LynxParachute2
  extends Rockets.Internal.PartialRocket;
  
    parameter SI.Duration start_delay = 0.5;
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation drogue_flange(animation = false, r = {0.5, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {20, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LinearAerodynamicsWithCanards.LinearAerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {30, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    
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
    Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzle_trans(animation = false, r = {-1.150, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-40, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  RocketControl.Components.Parts.SeparableFlange noseconeFlange(c = 100000, d = 1000,separation_duration = 0.3, separation_force = 500)  annotation(
    Placement(visible = true, transformation(origin = {-50, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Body nosecone(I_11 = 0.013576801, I_21 = 0, I_22 = 0.076867583, I_31 = 0, I_32 = 0, I_33 = 0.076867583, animation = false, enforceStates = false, m = 5.848, r_CM = {0.091, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fuselage_trans(animation = false, r = {-0.4, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nose_flange_trans(animation = false, r = {0.743, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit1(angle_1(fixed = true), angle_2(fixed = true), angle_3(fixed = true), r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_angle = true, use_r = true) annotation(
    Placement(visible = true, transformation(origin = {-31, 55}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition nosecone_pos annotation(
    Placement(visible = true, transformation(origin = {-20, 56}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition fuselage_pos annotation(
    Placement(visible = true, transformation(origin = {-2, 56}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition propellant_pos annotation(
    Placement(visible = true, transformation(origin = {-52, -54}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition main_pos annotation(
    Placement(visible = true, transformation(origin = {20, -84}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    
    SI.Mass total_mass;
    SI.Position center_of_mass[3];
  RocketControl.Components.Actuators.TFServoMotor fin_servo(a = {0.0769, 1}, saturation_angle(displayUnit = "deg") = 0.1745329251994329) annotation(
    Placement(visible = true, transformation(origin = {52, 40}, extent = {{-8, -8}, {8, 8}}, rotation = 180)));
  RocketControl.Components.Clocked.VectorHold vectorHold(n = 4) annotation(
    Placement(visible = true, transformation(origin = {74, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  total_mass = nosecone.m + fuselage.m + main.mass + drogue.mass + m2000r.m2000r.propellant.m;
  center_of_mass = (nosecone.m * nosecone_pos.r_rel + fuselage.m * fuselage_pos.r_rel + drogue.mass * drogue_pos.r_rel + main.mass * main_pos.r_rel + m2000r.m2000r.propellant.m * propellant_pos.r_rel) / total_mass;
  connect(lug_bow.frame_b, frame_lug_bow) annotation(
    Line(points = {{-80, -10}, {-100, -10}, {-100, 60}}, color = {95, 95, 95}));
  connect(lug_aft.frame_a, fuselage_trans.frame_a) annotation(
    Line(points = {{-60, -30}, {-10, -30}, {-10, 10}}));
  connect(fuselage.frame_a, fuselage_trans.frame_b) annotation(
    Line(points = {{-10, 80}, {-10, 30}}, color = {95, 95, 95}));
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
    Line(points = {{30, -36}, {60, -36}}, color = {95, 95, 95}));
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
    Line(points = {{-60, -30}, {-40, -30}, {-40, -40}}, color = {95, 95, 95}));
  connect(drogue_flange.frame_a, lug_aft.frame_a) annotation(
    Line(points = {{10, -36}, {-20, -36}, {-20, -30}, {-60, -30}}, color = {95, 95, 95}));
  connect(line_attachment_point.frame_a, lug_aft.frame_a) annotation(
    Line(points = {{10, -70}, {-20, -70}, {-20, -30}, {-60, -30}}, color = {95, 95, 95}));
  connect(main_flange.frame_a, lug_aft.frame_a) annotation(
    Line(points = {{10, -100}, {-20, -100}, {-20, -30}, {-60, -30}}, color = {95, 95, 95}));
  connect(nozzle_trans.frame_b, m2000r.frame_b) annotation(
    Line(points = {{-40, -60}, {-40, -70}}));
  connect(aerodynamics.frame_b, ref_center) annotation(
    Line(points = {{20, 80}, {6, 80}, {6, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(nose_flange_trans.frame_b, freeMotionScalarInit1.frame_a) annotation(
    Line(points = {{-50, 30}, {-50, 40}, {-31, 40}, {-31, 50}}));
  connect(freeMotionScalarInit1.frame_b, nosecone.frame_a) annotation(
    Line(points = {{-31, 60}, {-31, 72}, {-50, 72}, {-50, 80}}));
  connect(nosecone_pos.frame_b, nosecone.frame_a) annotation(
    Line(points = {{-20, 60}, {-20, 72}, {-50, 72}, {-50, 80}}));
  connect(nosecone_pos.frame_a, nose_flange_trans.frame_a) annotation(
    Line(points = {{-20, 52}, {-20, 36}, {-44, 36}, {-44, 10}, {-50, 10}}, color = {95, 95, 95}));
  connect(fuselage.frame_a, fuselage_pos.frame_b) annotation(
    Line(points = {{-10, 80}, {-10, 62}, {-2, 62}, {-2, 60}}, color = {95, 95, 95}));
  connect(fuselage_pos.frame_a, fuselage_trans.frame_a) annotation(
    Line(points = {{-2, 52}, {-2, 10}, {-10, 10}}, color = {95, 95, 95}));
  connect(nozzle_trans.frame_a, propellant_pos.frame_a) annotation(
    Line(points = {{-40, -40}, {-52, -40}, {-52, -50}}, color = {95, 95, 95}));
  connect(propellant_pos.frame_b, m2000r.frame_b) annotation(
    Line(points = {{-52, -58}, {-52, -70}, {-40, -70}}));
  connect(drogue_pos.frame_a, drogue_flange.frame_a) annotation(
    Line(points = {{16, -20}, {10, -20}, {10, -36}}));
  connect(main_pos.frame_b, main.chute) annotation(
    Line(points = {{24, -84}, {84, -84}, {84, -70}, {80, -70}}, color = {95, 95, 95}));
  connect(main_pos.frame_a, main_flange.frame_a) annotation(
    Line(points = {{16, -84}, {10, -84}, {10, -100}}));
  connect(vectorHold.y, fin_servo.setpoint) annotation(
    Line(points = {{70, 40}, {62, 40}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fin_servo.servo_pos, aerodynamics.finDeflection) annotation(
    Line(points = {{44, 40}, {12, 40}, {12, 74}, {20, 74}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.fin_setpoint, vectorHold.u) annotation(
    Line(points = {{100, 100}, {100, 40}, {78, 40}}, thickness = 0.5));
  connect(bus.main_deploy, main.enable) annotation(
    Line(points = {{100, 100}, {100, 10}, {70, 10}, {70, -60}}, color = {255, 0, 255}));
  connect(bus.drogue_deploy, drogue.enable) annotation(
    Line(points = {{100, 100}, {100, 10}, {70, 10}, {70, -30}}, color = {255, 0, 255}));
  connect(bus.drogue_deploy, noseconeFlange.enable) annotation(
    Line(points = {{100, 100}, {-80, 100}, {-80, 56}, {-59, 56}}, color = {255, 0, 255}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
  Diagram(graphics = {Text(origin = {-31, -21}, extent = {{-19, 7}, {19, -7}}, textString = "ref_center", textStyle = {TextStyle.Italic})}));
end LynxParachute2;
