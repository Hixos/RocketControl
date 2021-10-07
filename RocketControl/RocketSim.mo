within RocketControl;

model RocketSim
  import Modelica.Units.Conversions.from_deg;
  parameter SI.Mass m = 28;
  parameter SI.Distance s_max = 0.0001;
  parameter SI.ModulusOfElasticity c_x = Modelica.Constants.g_n * m / s_max;
  parameter SI.ModulusOfElasticity c_y = Modelica.Constants.g_n * m / s_max;
  parameter SI.ModulusOfElasticity c_z = 2500 / s_max;
  parameter SI.ModulusOfElasticity d_x = 2 * sqrt(c_x * m);
  parameter SI.ModulusOfElasticity d_y = 2 * sqrt(c_y * m) * 4;
  parameter SI.ModulusOfElasticity d_z = 2 * sqrt(c_z * m) * 4;
  Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.066590312, I_21 = -0.003190759, I_22 = 9.822815884, I_31 = 0.00128563, I_32 = -0.000234088, I_33 = 9.822815884, enforceStates = true, m = 22, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {50, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  RocketControl.Components.Motors.M2000R m2000r(start_delay = 0.5) annotation(
    Placement(visible = true, transformation(origin = {50, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzleTranslation(r = {-1.150, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  inner RocketControl.World.Atmosphere atmosphere annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(sequence = {3, 2, 1}) annotation(
    Placement(visible = true, transformation(origin = {90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(r = {-0.02, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(r = {-0.43, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 0, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 85, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
    Placement(visible = true, transformation(origin = {-30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1.2}) annotation(
    Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealAccelerometer idealAccelerometer annotation(
    Placement(visible = true, transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.MyWorld world(altitude_0 = 305, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.WithoutControl.Aerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealAccelerometer realAccelerometer(acc_max = 156.96, bias = {0.2, 0.4, -0.3}, bits = 16, fixedLocalSeed = {637914168, 1062993719, 2034216499}, samplePeriodMs = 20, sigmaBiasInstability = 0.01, sigmaNoise = 0.1) annotation(
    Placement(visible = true, transformation(origin = {90, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  when rocket.r_0[3] > 0 and time > 10 then
    terminate("Simulation terminated successfully");
  end when;
  connect(m2000r.frame_b, nozzleTranslation.frame_a) annotation(
    Line(points = {{50, -34}, {50, -20}}, color = {95, 95, 95}));
  connect(nozzleTranslation.frame_b, rocket.frame_a) annotation(
    Line(points = {{50, 0}, {50, 60}}));
  connect(absoluteAngles.frame_a, rocket.frame_a) annotation(
    Line(points = {{80, 10}, {50, 10}, {50, 60}}, color = {95, 95, 95}));
  connect(lug_bow.frame_a, rocket.frame_a) annotation(
    Line(points = {{20, 40}, {50, 40}, {50, 60}}, color = {95, 95, 95}));
  connect(lug_aft.frame_a, rocket.frame_a) annotation(
    Line(points = {{20, -20}, {20, 40}, {50, 40}, {50, 60}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, lug_bow.frame_b) annotation(
    Line(points = {{-20, 16}, {-20, 40}, {0, 40}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_aft, lug_aft.frame_b) annotation(
    Line(points = {{-20, 4}, {-10, 4}, {-10, -20}, {0, -20}}, color = {95, 95, 95}));
  connect(fixed.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-80, 10}, {-40, 10}}, color = {95, 95, 95}));
  connect(idealAccelerometer.frame_a, rocket.frame_a) annotation(
    Line(points = {{80, 90}, {66, 90}, {66, 50}, {50, 50}, {50, 60}}, color = {95, 95, 95}));
  connect(aerodynamics.frame_b, rocket.frame_a) annotation(
    Line(points = {{80, 50}, {50, 50}, {50, 60}}, color = {95, 95, 95}));
  connect(realAccelerometer.frame_a, rocket.frame_a) annotation(
    Line(points = {{80, -22}, {68, -22}, {68, 10}, {50, 10}, {50, 60}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 100, Tolerance = 0.0001, Interval = 0.01));
end RocketSim;
