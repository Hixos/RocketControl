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
    RocketControl.Components.Motors.M2000R m2000r(start_delay = 0.5) annotation(
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
      RocketControl.Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 0, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 84, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
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
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 0.0001, Interval = 0.01));
    end Simulation;

    model Sensors
    extends RocketControl.Interfaces.PartialAvionicsBusPort;
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
  RocketControl.Components.Sensors.RealMagnetometer realMagnetometer(b_max = 100e9, bias = {100, 200, -150}, bits = 16, fixedLocalSeed = {99, 432, 543543}, samplePeriodMs = 20, sigmaNoise = 10) annotation(
        Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealAccelerometer idealAccelerometer annotation(
        Placement(visible = true, transformation(origin = {-10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(sequence = {3, 2, 1}) annotation(
        Placement(visible = true, transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealMagnetometer idealMagnetometer annotation(
        Placement(visible = true, transformation(origin = {-10, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealGyroscope realGyroscope(bias = {3, -2, 1}, bits = 16, fixedLocalSeed = {9, 423, 43214321}, rate_max = 250, samplePeriodMs = 20, sigmaARW = 0.5, sigmaRRW = 1) annotation(
        Placement(visible = true, transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealAccelerometer realAccelerometer(acc_max = 156.96, bias = {0.2, 0.4, -0.3}, bits = 16, fixedLocalSeed = {637914168, 1062993719, 2034216499}, samplePeriodMs = 20, sigmaBiasInstability = 0.01, sigmaNoise = 0.1) annotation(
        Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealGyroscope idealGyroscope annotation(
        Placement(visible = true, transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Sensors.IdealBarometer idealBarometer annotation(
        Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealBarometer realBarometer(bias(displayUnit = "Pa") = 300, bits = 24, fixedLocalSeed = 5325587, p_max(displayUnit = "Pa") = 110000, p_min (displayUnit = "Pa") = 1000, samplePeriodMs = 50, sigmaNoise(displayUnit = "Pa") = 50)  annotation(
        Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
      connect(realAccelerometer.acc, bus.a) annotation(
        Line(points = {{0, 90}, {100, 90}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(realGyroscope.w_meas, bus.w) annotation(
        Line(points = {{0, 70}, {20, 70}, {20, 90}, {100, 90}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(realMagnetometer.b_meas, bus.b) annotation(
        Line(points = {{0, 50}, {20, 50}, {20, 90}, {100, 90}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealBarometer.frame_a, frame_a) annotation(
        Line(points = {{-20, -30}, {-40, -30}, {-40, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(realBarometer.frame_a, frame_a) annotation(
        Line(points = {{-20, 30}, {-40, 30}, {-40, 0}, {-100, 0}}));
  connect(realBarometer.press, bus.pressure) annotation(
        Line(points = {{0, 30}, {20, 30}, {20, 90}, {100, 90}, {100, 100}}, color = {0, 0, 127}));
    protected
      annotation(
        Icon(graphics = {Text(origin = {-1, 3}, extent = {{-79, 83}, {79, -83}}, textString = "Sens")}));
    end Sensors;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Lynx;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Rockets;
