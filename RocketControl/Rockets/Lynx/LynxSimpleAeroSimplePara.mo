within RocketControl.Rockets.Lynx;

model LynxSimpleAeroSimplePara
  extends Rockets.Internal.PartialRocket;
  outer RocketControl.World.SimOptions opt;
  parameter SI.Duration start_delay = 0.5;
  replaceable RocketControl.Rockets.Lynx.LinearAerodynamicsWithCanards.LinearAerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {70, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation to_parachute_link(animation = false, r = {0.61, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation to_lug_aft(animation = false, r = {-0.408, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation to_lug_bow(animation = false, r = {-0.009, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Body fuselage(I_11 = 0.06, I_21 = 0, I_22 = 6.436, I_31 = 0, I_32 = 0, I_33 = 6.436, animation = false, enforceStates = false, m = 18.362, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {-40, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzle_trans(animation = false, r = {-1.150, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-40, -48}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Components.Parachutes.SimpleParachute drogue(CLL_p = -5,Cd = 0.9, d = 0.15, initial_surface = 0.01, opening_delay = 0.5, opening_transient_duration = 0.2, surface = 0.6, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {60, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parachutes.SimpleParachute main(CLL_p = -5,Cd = 0.9, d = 0.15, initial_surface = 0.1, opening_delay = 1, opening_transient_duration = 0.7, surface = 10.6, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {80, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Actuators.TFServoMotor fin_servo(a = {0.0769, 1}) annotation(
    Placement(visible = true, transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Components.Propulsion.L1350 l1350(start_delay = start_delay)  annotation(
    Placement(visible = true, transformation(origin = {-40, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.SensorModels.RealFinPositionSensor fin_position_sens(noisy = true, samplePeriodMs = opt.samplePeriodMs, sigma_noise(displayUnit = "rad") = 0)  annotation(
    Placement(visible = true, transformation(origin = {12, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
//  for i in 1:4 loop
//    sample2.u[i] = der(deflection2Control.control[i]);
//  end for;
  connect(to_lug_bow.frame_b, frame_lug_bow) annotation(
    Line(points = {{-80, -10}, {-100, -10}, {-100, 60}}, color = {95, 95, 95}));
  connect(ref_center, to_lug_aft.frame_a) annotation(
    Line(points = {{100, 0}, {-10, 0}, {-10, -30}, {-60, -30}}));
  connect(to_lug_aft.frame_b, frame_lug_aft) annotation(
    Line(points = {{-80, -30}, {-100, -30}, {-100, -40}}, color = {95, 95, 95}));
  connect(to_lug_aft.frame_a, to_lug_bow.frame_a) annotation(
    Line(points = {{-60, -30}, {-50, -30}, {-50, -10}, {-60, -10}}, color = {95, 95, 95}));
  connect(to_lug_aft.frame_a, nozzle_trans.frame_a) annotation(
    Line(points = {{-60, -30}, {-40, -30}, {-40, -38}}, color = {95, 95, 95}));
  connect(to_parachute_link.frame_a, to_lug_aft.frame_a) annotation(
    Line(points = {{0, -60}, {-20, -60}, {-20, -30}, {-60, -30}}, color = {95, 95, 95}));
  connect(aerodynamics.frame_b, ref_center) annotation(
    Line(points = {{60, 80}, {28, 80}, {28, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(bus.drogue_deploy, drogue.enable) annotation(
    Line(points = {{100, 100}, {100, 20}, {60, 20}, {60, -21}}, color = {255, 0, 255}));
  connect(to_parachute_link.frame_b, drogue.frame_a) annotation(
    Line(points = {{20, -60}, {40, -60}, {40, -30}, {50, -30}}, color = {95, 95, 95}));
  connect(to_parachute_link.frame_b, main.frame_a) annotation(
    Line(points = {{20, -60}, {40, -60}, {40, -70}, {70, -70}}, color = {95, 95, 95}));
  connect(bus.main_deploy, main.enable) annotation(
    Line(points = {{100, 100}, {100, 20}, {80, 20}, {80, -61}}, color = {255, 0, 255}));
  connect(nozzle_trans.frame_a, fuselage.frame_a) annotation(
    Line(points = {{-40, -38}, {-40, 60}}));
  connect(bus.fin_setpoint, fin_servo.setpoint) annotation(
    Line(points = {{100, 100}, {100, 50}, {82, 50}}, thickness = 0.5));
  connect(nozzle_trans.frame_b, l1350.frame_b) annotation(
    Line(points = {{-40, -58}, {-40, -72}}));
  connect(fin_servo.servo_pos, fin_position_sens.fin_pos_true) annotation(
    Line(points = {{60, 50}, {12, 50}, {12, 60}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fin_position_sens.control_meas, bus.control_position_meas) annotation(
    Line(points = {{12, 84}, {12, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fin_servo.servo_pos, aerodynamics.finDeflection) annotation(
    Line(points = {{60, 50}, {36, 50}, {36, 74}, {60, 74}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.drogue_deploy, aerodynamics.chute_open) annotation(
    Line(points = {{100, 100}, {60, 100}, {60, 88}}, color = {255, 0, 255}));
protected
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
    Diagram(graphics = {Text(origin = {-31, -21}, extent = {{-19, 7}, {19, -7}}, textString = "ref_center", textStyle = {TextStyle.Italic})}));
end LynxSimpleAeroSimplePara;
