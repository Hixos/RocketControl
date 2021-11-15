within RocketControl;

model Simulations
  model Lynx
  import Modelica.Units.Conversions.from_deg;
  import RocketControl.Rockets.Lynx.*;
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
    RocketControl.Components.Parts.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 130, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 84, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.Aerodynamics.Aerodynamics aerodynamics annotation(
      Placement(visible = true, transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Parts.LandDetector landDetector annotation(
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
  end Lynx;

  model LynxWithCanards
  import Modelica.Units.Conversions.from_deg;
    import RocketControl.Rockets.Lynx.*;
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
    Components.Parts.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 130, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 84, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Components.Parts.LandDetector landDetector annotation(
      Placement(visible = true, transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.LynxAirframe lynxAirframe annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.Sensors sensors annotation(
      Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Rockets.Lynx.Estimators estimators annotation(
      Placement(visible = true, transformation(origin = {30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
      Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    GNC.PitchController pitchController annotation(
      Placement(visible = true, transformation(origin = {10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = -4) annotation(
      Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
      Line(points = {{40, -30}, {40, -70}}));
    connect(lynxAirframe.ref_center, aerodynamics.frame_b) annotation(
      Line(points = {{0, 10}, {60, 10}, {60, 30}, {80, 30}}));
    connect(pitchController.finDeflection, aerodynamics.finDeflection) annotation(
      Line(points = {{20, 70}, {68, 70}, {68, 24}, {80, 24}}, thickness = 0.5));
    connect(const.y, pitchController.u) annotation(
      Line(points = {{-59, 70}, {0, 70}}, color = {0, 0, 127}));
    annotation(
      experiment(StartTime = 0, StopTime = 100, Tolerance = 0.0001, Interval = 0.01),
      Icon(graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}, coordinateSystem(grid = {2, 0})));
  end LynxWithCanards;
equation

annotation(
    Icon(coordinateSystem(grid = {2, 0}), graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}));
end Simulations;
