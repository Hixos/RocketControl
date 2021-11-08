within RocketControl;

package Rockets
  package Internal
    model PartialAirframe
    extends Components.LaunchPad.PartialLaunchMount;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a ref_center annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    equation

      annotation(
        Icon(graphics = {Text(origin = {467.111, 32}, lineColor = {128, 128, 128}, extent = {{-423.111, -41}, {-311.111, -82}}, textString = "ref_center")}));
    end PartialAirframe;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Internal;

  package Lynx
    model LynxAirframe
    extends RocketControl.Rockets.Internal.PartialAirframe;
    Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.06, I_21 = 0, I_22 = 6.436, I_31 = 0, I_32 = 0, I_33 = 6.437, enforceStates = false, m = 18.362, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
        Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(r = {-0.02, 0, -0.075}) annotation(
        Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(r = {-0.43, 0, -0.075}) annotation(
        Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    RocketControl.Components.Motors.M2000R m2000r(start_delay = 2) annotation(
        Placement(visible = true, transformation(origin = {30, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzleTranslation(r = {-1.150, 0, 0}) annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    equation
      connect(ref_center, rocket.frame_a) annotation(
        Line(points = {{100, 0}, {30, 0}, {30, 60}}));
      connect(lug_bow.frame_a, rocket.frame_a) annotation(
        Line(points = {{-40, 40}, {30, 40}, {30, 60}}, color = {95, 95, 95}));
      connect(lug_bow.frame_b, frame_a) annotation(
        Line(points = {{-60, 40}, {-74, 40}, {-74, 60}, {-100, 60}}, color = {95, 95, 95}));
      connect(frame_a1, lug_aft.frame_b) annotation(
        Line(points = {{-100, -40}, {-60, -40}}));
      connect(lug_aft.frame_a, rocket.frame_a) annotation(
        Line(points = {{-40, -40}, {0, -40}, {0, 40}, {30, 40}, {30, 60}}));
      connect(m2000r.frame_b, nozzleTranslation.frame_a) annotation(
        Line(points = {{30, -54.2}, {30, -40.2}}, color = {95, 95, 95}));
    connect(nozzleTranslation.frame_b, rocket.frame_a) annotation(
        Line(points = {{30, -20}, {30, 60}}, color = {95, 95, 95}));
      annotation(
        Icon(graphics = {Polygon(lineColor = {60, 60, 61}, fillColor = {97, 183, 229}, fillPattern = FillPattern.VerticalCylinder, points = {{-20, 60}, {0, 100}, {20, 60}, {20, -60}, {40, -100}, {-40, -100}, {-20, -60}, {-20, 60}}), Text(origin = {-6, 16},lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name")}));
    end LynxAirframe;

    model Simulation
    import Modelica.Units.Conversions.from_deg;
      parameter SI.Mass m = 28;
      parameter SI.Distance s_max = 0.0001;
      parameter SI.ModulusOfElasticity c_x = Modelica.Constants.g_n * m / s_max;
      parameter SI.ModulusOfElasticity c_y = Modelica.Constants.g_n * m / s_max;
      parameter SI.ModulusOfElasticity c_z = 2500 / s_max;
      parameter SI.ModulusOfElasticity d_x = 2 * sqrt(c_x * m);
      parameter SI.ModulusOfElasticity d_y = 2 * sqrt(c_y * m) * 4;
      parameter SI.ModulusOfElasticity d_z = 2 * sqrt(c_z * m) * 4;
      Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1.2}) annotation(
        Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 130, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 84, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
        Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
        Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Aerodynamics.WithoutControl.Aerodynamics aerodynamics annotation(
        Placement(visible = true, transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.LandDetector landDetector annotation(
        Placement(visible = true, transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Rockets.Lynx.LynxAirframe lynxAirframe annotation(
        Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Rockets.Lynx.Sensors sensors annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.Estimators estimators annotation(
        Placement(visible = true, transformation(origin = {30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(fixed.frame_b, launchRail.frame_a) annotation(
        Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
    connect(launchRail.frame_b_lug_bow, lynxAirframe.frame_a) annotation(
        Line(points = {{-40, 16}, {-20, 16}}));
    connect(launchRail.frame_b_lug_aft, lynxAirframe.frame_a1) annotation(
        Line(points = {{-40, 4}, {-20, 4}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, aerodynamics.frame_b) annotation(
        Line(points = {{0, 10}, {60, 10}, {60, 50}, {80, 50}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, landDetector.frame_a) annotation(
        Line(points = {{0, 10}, {60, 10}, {60, -90}, {80, -90}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, sensors.frame_a) annotation(
        Line(points = {{0, 10}, {10, 10}, {10, -30}, {20, -30}}, color = {95, 95, 95}));
  connect(sensors.bus, estimators.bus) annotation(
        Line(points = {{40, -20}, {40, -60}}));
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 0.0001, Interval = 0.01),
        Icon(graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}));
    end Simulation;

    model Sensors
     extends Modelica.Icons.RoundSensor;
    extends RocketControl.Interfaces.PartialAvionicsBusPort;
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
  RocketControl.Components.Sensors.RealMagnetometer realMagnetometer(b_max = 2500e3, bias = 350, bits = 16, fixedLocalSeed = {99, 432, 543543}, misalignement = {0.2, -0.1, 0.3}, samplePeriodMs = 20, sigmaNoise = 10) annotation(
        Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealAccelerometer idealAccelerometer annotation(
        Placement(visible = true, transformation(origin = {-10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(sequence = {3, 2, 1}) annotation(
        Placement(visible = true, transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealMagnetometer idealMagnetometer annotation(
        Placement(visible = true, transformation(origin = {-10, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealGyroscope realGyroscope(bias = {0.1, -0.05, -0.02}, bits = 16, fixedLocalSeed = {9, 423, 43214321}, rate_max = 250, samplePeriodMs = 20, sigmaARW = 0.5, sigmaRRW = 1) annotation(
        Placement(visible = true, transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealAccelerometer realAccelerometer(acc_max = 156.96, bias = {0.02, 0.04, -0.03}, bits = 16, fixedLocalSeed = {637914168, 1062993719, 2034216499}, samplePeriodMs = 20, sigmaBiasInstability = 0.01, sigmaNoise = 0.1) annotation(
        Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealGyroscope idealGyroscope annotation(
        Placement(visible = true, transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Sensors.IdealBarometer idealBarometer annotation(
        Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealBarometer realBarometer(bias(displayUnit = "Pa") = 300, bits = 24, fixedLocalSeed = 5325587, p_max(displayUnit = "Pa") = 110000, p_min (displayUnit = "Pa") = 1000, samplePeriodMs = 50, sigmaNoise(displayUnit = "Pa") = 50)  annotation(
        Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)  annotation(
        Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealGNSS realGNSS(fixedLocalSeed = {1775435783, 568478634, -1728550798},samplePeriodMs = 20, sigmaNoise_xy = 5, sigmaNoise_z = 10)  annotation(
        Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion quat_true annotation(
        Placement(visible = true, transformation(origin = {28, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler eul_true annotation(
        Placement(visible = true, transformation(origin = {68, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Math.Blocks.UnwrapAngle unwrapAngle(internalClock = true, n = 3)  annotation(
        Placement(visible = true, transformation(origin = {116, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
    
      connect(frame_a, realMagnetometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, 50}, {-20, 50}}));
      connect(frame_a, realGyroscope.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, 70}, {-20, 70}}));
      connect(frame_a, realAccelerometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, 90}, {-20, 90}}));
      connect(frame_a, absoluteAngles.frame_a) annotation(
        Line(points = {{-100, 0}, {-90, 0}, {-90, 30}, {-80, 30}}));
      connect(frame_a, idealAccelerometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -50}, {-20, -50}}));
      connect(frame_a, idealGyroscope.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -70}, {-20, -70}}));
      connect(frame_a, idealMagnetometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -90}, {-20, -90}}));
      connect(realAccelerometer.acc, bus.a_meas) annotation(
        Line(points = {{0, 90}, {20.5, 90}, {20.5, 100}, {102, 100}, {102, 99}, {100, 99}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(realGyroscope.w_meas, bus.w_meas) annotation(
        Line(points = {{0, 70}, {20, 70}, {20, 100}, {102, 100}, {102, 97}, {100, 97}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(realMagnetometer.b_meas, bus.b_meas) annotation(
        Line(points = {{0, 50}, {20, 50}, {20, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealBarometer.frame_a, frame_a) annotation(
        Line(points = {{-20, -30}, {-40, -30}, {-40, 0}, {-100, 0}}, color = {95, 95, 95}));
      connect(realBarometer.frame_a, frame_a) annotation(
        Line(points = {{-20, 30}, {-40, 30}, {-40, 0}, {-100, 0}}));
      connect(realBarometer.press, bus.p_meas) annotation(
        Line(points = {{0, 30}, {20, 30}, {20, 100}, {102, 100}, {102, 98}, {100, 98}, {100, 100}}, color = {0, 0, 127}));
      connect(frame_a, absolutePosition.frame_a) annotation(
        Line(points = {{-100, 0}, {-90, 0}, {-90, -70}, {-80, -70}}));
      connect(realGNSS.frame_a, frame_a) annotation(
        Line(points = {{-20, 10}, {-59, 10}, {-59, 0}, {-100, 0}}, color = {95, 95, 95}));
      connect(realGNSS.pos, bus.x_meas) annotation(
        Line(points = {{0, 10}, {20, 10}, {20, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(frame_a, quat_true.frame_a) annotation(
        Line(points = {{-100, 0}, {18, 0}, {18, -2}}));
      connect(quat_true.q, eul_true.q) annotation(
        Line(points = {{38, -2}, {58, -2}}, color = {0, 0, 127}));
  connect(unwrapAngle.u, eul_true.eul) annotation(
        Line(points = {{104, -2}, {78, -2}}, color = {0, 0, 127}));
    protected
      annotation(
        Icon(graphics = {Line(origin = {-83, 0}, points = {{-13, 0}, {13, 0}}), Text(origin = {-1.42109e-14, -40}, extent = {{-60, 20}, {60, -20}}, textString = "sens"), Line(origin = {85, 45}, points = {{-15, -45}, {15, -45}, {15, 45}}), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
    end Sensors;

    model Estimators
    extends RocketControl.Interfaces.Internal.Icons.Estimator;
    extends RocketControl.Interfaces.PartialAvionicsBusPort;
      RocketControl.Components.Control.Estimators.AES aes(elevation0 = 84, heading0 = 130, samplingPeriodMs = 20, sigma_b = 2, sigma_u = from_deg(10), sigma_v = from_deg(60)) annotation(
        Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler eul_est annotation(
        Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.UnwrapAngle unwrapAngle(n = 3)  annotation(
        Placement(visible = true, transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(bus.w_meas, aes.w_meas_degs) annotation(
        Line(points = {{100, 100}, {-100, 100}, {-100, 56}, {-60, 56}}, thickness = 0.5));
      connect(bus.b_meas, aes.b_meas_nt) annotation(
        Line(points = {{100, 100}, {-100, 100}, {-100, 50}, {-60, 50}}, thickness = 0.5));
      connect(bus.x_meas, aes.r_0_est) annotation(
        Line(points = {{100, 100}, {-100, 100}, {-100, 44}, {-60, 44}}, thickness = 0.5));
      connect(aes.q_est, bus.q_est) annotation(
        Line(points = {{-38, 56}, {-30, 56}, {-30, 52}, {100, 52}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(aes.w_est_degs, bus.w_est) annotation(
        Line(points = {{-38, 46}, {-30, 46}, {-30, 52}, {100, 52}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
  connect(aes.q_est, eul_est.q) annotation(
        Line(points = {{-39, 55}, {-30, 55}, {-30, 30}, {-2, 30}}, color = {0, 0, 127}));
  connect(eul_est.eul, unwrapAngle.u) annotation(
        Line(points = {{21, 30}, {38, 30}}, color = {0, 0, 127}));
      annotation(
        Icon,
        Diagram);
    end Estimators;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Lynx;
  
  package LynxWithCanards
    model LynxAirframe
    extends RocketControl.Rockets.Internal.PartialAirframe;
    Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.06, I_21 = 0, I_22 = 6.436, I_31 = 0, I_32 = 0, I_33 = 6.437, enforceStates = false, m = 18.362, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
        Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(r = {-0.02, 0, -0.075}) annotation(
        Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(r = {-0.43, 0, -0.075}) annotation(
        Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    RocketControl.Components.Motors.M2000R m2000r(start_delay = 2) annotation(
        Placement(visible = true, transformation(origin = {30, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzleTranslation(r = {-1.150, 0, 0}) annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    equation
      connect(ref_center, rocket.frame_a) annotation(
        Line(points = {{100, 0}, {30, 0}, {30, 60}}));
      connect(lug_bow.frame_a, rocket.frame_a) annotation(
        Line(points = {{-40, 40}, {30, 40}, {30, 60}}, color = {95, 95, 95}));
      connect(lug_bow.frame_b, frame_a) annotation(
        Line(points = {{-60, 40}, {-74, 40}, {-74, 60}, {-100, 60}}, color = {95, 95, 95}));
      connect(frame_a1, lug_aft.frame_b) annotation(
        Line(points = {{-100, -40}, {-60, -40}}));
      connect(lug_aft.frame_a, rocket.frame_a) annotation(
        Line(points = {{-40, -40}, {0, -40}, {0, 40}, {30, 40}, {30, 60}}));
      connect(m2000r.frame_b, nozzleTranslation.frame_a) annotation(
        Line(points = {{30, -54.2}, {30, -40.2}}, color = {95, 95, 95}));
    connect(nozzleTranslation.frame_b, rocket.frame_a) annotation(
        Line(points = {{30, -20}, {30, 60}}, color = {95, 95, 95}));
      annotation(
        Icon(graphics = {Polygon(lineColor = {60, 60, 61}, fillColor = {97, 183, 229}, fillPattern = FillPattern.VerticalCylinder, points = {{-20, 60}, {0, 100}, {20, 60}, {20, -60}, {40, -100}, {-40, -100}, {-20, -60}, {-20, 60}}), Text(origin = {-6, 16},lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name")}));
    end LynxAirframe;
  
    model Simulation
    import Modelica.Units.Conversions.from_deg;
      parameter SI.Mass m = 28;
      parameter SI.Distance s_max = 0.0001;
      parameter SI.ModulusOfElasticity c_x = Modelica.Constants.g_n * m / s_max;
      parameter SI.ModulusOfElasticity c_y = Modelica.Constants.g_n * m / s_max;
      parameter SI.ModulusOfElasticity c_z = 2500 / s_max;
      parameter SI.ModulusOfElasticity d_x = 2 * sqrt(c_x * m);
      parameter SI.ModulusOfElasticity d_y = 2 * sqrt(c_y * m) * 4;
      parameter SI.ModulusOfElasticity d_z = 2 * sqrt(c_z * m) * 4;
      Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1.2}) annotation(
        Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 130, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 84, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
        Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
        Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.LandDetector landDetector annotation(
        Placement(visible = true, transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Rockets.Lynx.LynxAirframe lynxAirframe annotation(
        Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Rockets.Lynx.Sensors sensors annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.Estimators estimators annotation(
        Placement(visible = true, transformation(origin = {30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.WithControl.Aerodynamics aerodynamics annotation(
        Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Control.PitchController pitchController annotation(
        Placement(visible = true, transformation(origin = {18, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = -4)  annotation(
        Placement(visible = true, transformation(origin = {-74, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(fixed.frame_b, launchRail.frame_a) annotation(
        Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
    connect(launchRail.frame_b_lug_bow, lynxAirframe.frame_a) annotation(
        Line(points = {{-40, 16}, {-20, 16}}));
    connect(launchRail.frame_b_lug_aft, lynxAirframe.frame_a1) annotation(
        Line(points = {{-40, 4}, {-20, 4}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, landDetector.frame_a) annotation(
        Line(points = {{0, 10}, {60, 10}, {60, -90}, {80, -90}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, sensors.frame_a) annotation(
        Line(points = {{0, 10}, {10, 10}, {10, -30}, {20, -30}}, color = {95, 95, 95}));
  connect(sensors.bus, estimators.bus) annotation(
        Line(points = {{40, -20}, {40, -60}}));
  connect(lynxAirframe.ref_center, aerodynamics.frame_b) annotation(
        Line(points = {{0, 10}, {60, 10}, {60, 30}, {80, 30}}));
  connect(pitchController.finDeflectionOutput, aerodynamics.finDeflectionInput) annotation(
        Line(points = {{28, 74}, {68, 74}, {68, 24}, {80, 24}}, thickness = 0.5));
  connect(const.y, pitchController.u) annotation(
        Line(points = {{-62, 72}, {8, 72}, {8, 74}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 0.0001, Interval = 0.01),
        Icon(graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}, coordinateSystem(grid = {2, 0})));
    end Simulation;
  
    model Sensors
     extends Modelica.Icons.RoundSensor;
    extends RocketControl.Interfaces.PartialAvionicsBusPort;
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
  RocketControl.Components.Sensors.RealMagnetometer realMagnetometer(b_max = 2500e3, bias = 350, bits = 16, fixedLocalSeed = {99, 432, 543543}, misalignement = {0.2, -0.1, 0.3}, samplePeriodMs = 20, sigmaNoise = 10) annotation(
        Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealAccelerometer idealAccelerometer annotation(
        Placement(visible = true, transformation(origin = {-10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(sequence = {3, 2, 1}) annotation(
        Placement(visible = true, transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealMagnetometer idealMagnetometer annotation(
        Placement(visible = true, transformation(origin = {-10, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealGyroscope realGyroscope(bias = {0.1, -0.05, -0.02}, bits = 16, fixedLocalSeed = {9, 423, 43214321}, rate_max = 250, samplePeriodMs = 20, sigmaARW = 0.5, sigmaRRW = 1) annotation(
        Placement(visible = true, transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealAccelerometer realAccelerometer(acc_max = 156.96, bias = {0.02, 0.04, -0.03}, bits = 16, fixedLocalSeed = {637914168, 1062993719, 2034216499}, samplePeriodMs = 20, sigmaBiasInstability = 0.01, sigmaNoise = 0.1) annotation(
        Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealGyroscope idealGyroscope annotation(
        Placement(visible = true, transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Sensors.IdealBarometer idealBarometer annotation(
        Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealBarometer realBarometer(bias(displayUnit = "Pa") = 300, bits = 24, fixedLocalSeed = 5325587, p_max(displayUnit = "Pa") = 110000, p_min (displayUnit = "Pa") = 1000, samplePeriodMs = 50, sigmaNoise(displayUnit = "Pa") = 50)  annotation(
        Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)  annotation(
        Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealGNSS realGNSS(fixedLocalSeed = {1775435783, 568478634, -1728550798},samplePeriodMs = 20, sigmaNoise_xy = 5, sigmaNoise_z = 10)  annotation(
        Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion quat_true annotation(
        Placement(visible = true, transformation(origin = {28, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler eul_true annotation(
        Placement(visible = true, transformation(origin = {68, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Math.Blocks.UnwrapAngle unwrapAngle(internalClock = true, n = 3)  annotation(
        Placement(visible = true, transformation(origin = {116, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
    
      connect(frame_a, realMagnetometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, 50}, {-20, 50}}));
      connect(frame_a, realGyroscope.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, 70}, {-20, 70}}));
      connect(frame_a, realAccelerometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, 90}, {-20, 90}}));
      connect(frame_a, absoluteAngles.frame_a) annotation(
        Line(points = {{-100, 0}, {-90, 0}, {-90, 30}, {-80, 30}}));
      connect(frame_a, idealAccelerometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -50}, {-20, -50}}));
      connect(frame_a, idealGyroscope.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -70}, {-20, -70}}));
      connect(frame_a, idealMagnetometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -90}, {-20, -90}}));
      connect(realAccelerometer.acc, bus.a_meas) annotation(
        Line(points = {{0, 90}, {20.5, 90}, {20.5, 100}, {102, 100}, {102, 99}, {100, 99}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(realGyroscope.w_meas, bus.w_meas) annotation(
        Line(points = {{0, 70}, {20, 70}, {20, 100}, {102, 100}, {102, 97}, {100, 97}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(realMagnetometer.b_meas, bus.b_meas) annotation(
        Line(points = {{0, 50}, {20, 50}, {20, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealBarometer.frame_a, frame_a) annotation(
        Line(points = {{-20, -30}, {-40, -30}, {-40, 0}, {-100, 0}}, color = {95, 95, 95}));
      connect(realBarometer.frame_a, frame_a) annotation(
        Line(points = {{-20, 30}, {-40, 30}, {-40, 0}, {-100, 0}}));
      connect(realBarometer.press, bus.p_meas) annotation(
        Line(points = {{0, 30}, {20, 30}, {20, 100}, {102, 100}, {102, 98}, {100, 98}, {100, 100}}, color = {0, 0, 127}));
      connect(frame_a, absolutePosition.frame_a) annotation(
        Line(points = {{-100, 0}, {-90, 0}, {-90, -70}, {-80, -70}}));
      connect(realGNSS.frame_a, frame_a) annotation(
        Line(points = {{-20, 10}, {-59, 10}, {-59, 0}, {-100, 0}}, color = {95, 95, 95}));
      connect(realGNSS.pos, bus.x_meas) annotation(
        Line(points = {{0, 10}, {20, 10}, {20, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(frame_a, quat_true.frame_a) annotation(
        Line(points = {{-100, 0}, {18, 0}, {18, -2}}));
      connect(quat_true.q, eul_true.q) annotation(
        Line(points = {{38, -2}, {58, -2}}, color = {0, 0, 127}));
  connect(unwrapAngle.u, eul_true.eul) annotation(
        Line(points = {{104, -2}, {78, -2}}, color = {0, 0, 127}));
    protected
      annotation(
        Icon(graphics = {Line(origin = {-83, 0}, points = {{-13, 0}, {13, 0}}), Text(origin = {-1.42109e-14, -40}, extent = {{-60, 20}, {60, -20}}, textString = "sens"), Line(origin = {85, 45}, points = {{-15, -45}, {15, -45}, {15, 45}}), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
    end Sensors;
  
    model Estimators
    extends RocketControl.Interfaces.Internal.Icons.Estimator;
    extends RocketControl.Interfaces.PartialAvionicsBusPort;
      RocketControl.Components.Control.Estimators.AES aes(elevation0 = 84, heading0 = 130, samplingPeriodMs = 20, sigma_b = 2, sigma_u = from_deg(10), sigma_v = from_deg(60)) annotation(
        Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler eul_est annotation(
        Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.UnwrapAngle unwrapAngle(n = 3)  annotation(
        Placement(visible = true, transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(bus.w_meas, aes.w_meas_degs) annotation(
        Line(points = {{100, 100}, {-100, 100}, {-100, 56}, {-60, 56}}, thickness = 0.5));
      connect(bus.b_meas, aes.b_meas_nt) annotation(
        Line(points = {{100, 100}, {-100, 100}, {-100, 50}, {-60, 50}}, thickness = 0.5));
      connect(bus.x_meas, aes.r_0_est) annotation(
        Line(points = {{100, 100}, {-100, 100}, {-100, 44}, {-60, 44}}, thickness = 0.5));
      connect(aes.q_est, bus.q_est) annotation(
        Line(points = {{-38, 56}, {-30, 56}, {-30, 52}, {100, 52}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(aes.w_est_degs, bus.w_est) annotation(
        Line(points = {{-38, 46}, {-30, 46}, {-30, 52}, {100, 52}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
  connect(aes.q_est, eul_est.q) annotation(
        Line(points = {{-39, 55}, {-30, 55}, {-30, 30}, {-2, 30}}, color = {0, 0, 127}));
  connect(eul_est.eul, unwrapAngle.u) annotation(
        Line(points = {{21, 30}, {38, 30}}, color = {0, 0, 127}));
      annotation(
        Icon,
        Diagram);
    end Estimators;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end LynxWithCanards;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Rockets;
