within RocketControl;

model RocketSimWithControl
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
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzleTranslation(r = {-1.150, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {50, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  inner RocketControl.World.Atmosphere atmosphere annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.MyWorld world(n = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(sequence = {3, 2, 1}) annotation(
    Placement(visible = true, transformation(origin = {90, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(r = {-0.02, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(r = {-0.43, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {10, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 0, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 45, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
    Placement(visible = true, transformation(origin = {-40, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1.2}) annotation(
    Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Control.PitchController pitchController annotation(
    Placement(visible = true, transformation(origin = {2, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 6.5) annotation(
    Placement(visible = true, transformation(origin = {80, -88}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Math.Add err(k1 = -1, k2 = 1) annotation(
    Placement(visible = true, transformation(origin = {6, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax = 10) annotation(
    Placement(visible = true, transformation(origin = {-44, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.UnitConversions.To_deg to_deg annotation(
    Placement(visible = true, transformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Continuous.PID pid(Td = 0.1, Ti = 10, k = -2) annotation(
    Placement(visible = true, transformation(origin = {-88, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.WithControl.Aerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {90, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Motors.M2000R m2000r(start_delay = 0.5) annotation(
    Placement(visible = true, transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Sensors.RealAccelerometer realAccelerometer(acc_max = 150, bias = {0, 0, 0}, bits = 16, samplePeriodMs = 20, sigmaBiasInstability = 0.03, sigmaNoise = 0.3)  annotation(
    Placement(visible = true, transformation(origin = {164, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  when rocket.r_0[3] > 0 and time > 10 then
    terminate("Simulation terminated successfully");
  end when;
  connect(nozzleTranslation.frame_b, rocket.frame_a) annotation(
    Line(points = {{50, 2}, {50, 60}}));
  connect(absoluteAngles.frame_a, rocket.frame_a) annotation(
    Line(points = {{80, 8}, {50, 8}, {50, 60}}, color = {95, 95, 95}));
  connect(lug_bow.frame_a, rocket.frame_a) annotation(
    Line(points = {{20, 40}, {50, 40}, {50, 60}}, color = {95, 95, 95}));
  connect(lug_aft.frame_a, rocket.frame_a) annotation(
    Line(points = {{20, 4}, {20, 40}, {50, 40}, {50, 60}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, lug_bow.frame_b) annotation(
    Line(points = {{-30, 16}, {-30, 40}, {0, 40}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_aft, lug_aft.frame_b) annotation(
    Line(points = {{-30, 4}, {0, 4}}, color = {95, 95, 95}));
  connect(fixed.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-80, 10}, {-50, 10}}, color = {95, 95, 95}));
  connect(const.y, err.u1) annotation(
    Line(points = {{70, -88}, {20, -88}, {20, -92}, {18, -92}}, color = {0, 0, 127}));
  connect(pitchController.u, limiter.y) annotation(
    Line(points = {{-8, -38}, {-33, -38}}, color = {0, 0, 127}));
  connect(absoluteAngles.angles[2], to_deg.u) annotation(
    Line(points = {{102, 8}, {102, 7}, {106, 7}, {106, 8}, {110, 8}, {110, -18}}, color = {0, 0, 127}));
  connect(to_deg.y, err.u2) annotation(
    Line(points = {{110, -41}, {110, -80}, {18, -80}}, color = {0, 0, 127}));
  connect(pid.y, limiter.u) annotation(
    Line(points = {{-76, -38}, {-56, -38}}, color = {0, 0, 127}));
  connect(pid.u, err.y) annotation(
    Line(points = {{-100, -38}, {-124, -38}, {-124, -86}, {-4, -86}}, color = {0, 0, 127}));
  connect(aerodynamics.frame_b, rocket.frame_a) annotation(
    Line(points = {{80, 60}, {64, 60}, {64, 40}, {50, 40}, {50, 60}}, color = {95, 95, 95}));
  connect(aerodynamics.finDeflectionInput, pitchController.finDeflectionOutput) annotation(
    Line(points = {{80, 53}, {66, 53}, {66, -38}, {12, -38}}, thickness = 0.5));
  connect(nozzleTranslation.frame_a, m2000r.frame_b) annotation(
    Line(points = {{50, -18}, {50, -40}}, color = {95, 95, 95}));
  connect(realAccelerometer.frame_a, rocket.frame_a) annotation(
    Line(points = {{154, 38}, {50, 38}, {50, 60}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-4, Interval = 0.01));
end RocketSimWithControl;
