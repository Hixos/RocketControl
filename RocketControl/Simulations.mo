within RocketControl;

package Simulations
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
    RocketControl.Components.Parts.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 2.268928027592628, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 1.48352986419518, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Parts.LandDetector landDetector annotation(
      Placement(visible = true, transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LynxAirframe lynxAirframe annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.SensorsIdeal sensorsIdeal annotation(
      Placement(visible = true, transformation(origin = {-30, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Rockets.Lynx.Aerodynamics.Aerodynamics aerodynamics annotation(
      Placement(visible = true, transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.Estimators estimators annotation(
      Placement(visible = true, transformation(origin = {-30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.Errors errors annotation(
      Placement(visible = true, transformation(origin = {30, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.Sensors sensors annotation(
      Placement(visible = true, transformation(origin = {-30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Blocks.EulerRates eulerRates annotation(
      Placement(visible = true, transformation(origin = {18, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Rockets.Lynx.SensorsIdealCont sensorsIdealCont annotation(
      Placement(visible = true, transformation(origin = {-48, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(fixed.frame_b, launchRail.frame_a) annotation(
      Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
    connect(launchRail.frame_b_lug_bow, lynxAirframe.frame_a) annotation(
      Line(points = {{-40, 16}, {-20, 16}}));
    connect(launchRail.frame_b_lug_aft, lynxAirframe.frame_a1) annotation(
      Line(points = {{-40, 4}, {-20, 4}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, landDetector.frame_a) annotation(
      Line(points = {{0, 10}, {60, 10}, {60, -90}, {80, -90}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, sensorsIdeal.frame_a) annotation(
      Line(points = {{0, 10}, {10, 10}, {10, -12}, {-52, -12}, {-52, -28}, {-40, -28}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, aerodynamics.frame_b) annotation(
      Line(points = {{0, 10}, {60, 10}, {60, 50}, {80, 50}}, color = {95, 95, 95}));
  connect(errors.frame_a, lynxAirframe.ref_center) annotation(
      Line(points = {{20, -50}, {10, -50}, {10, 10}, {0, 10}}, color = {95, 95, 95}));
  connect(sensors.frame_a, lynxAirframe.ref_center) annotation(
      Line(points = {{-40, -60}, {-52, -60}, {-52, -12}, {10, -12}, {10, 10}, {0, 10}}, color = {95, 95, 95}));
  connect(sensors.bus, estimators.bus) annotation(
      Line(points = {{-20, -60}, {-6, -60}, {-6, -90}, {-20, -90}}, thickness = 0.5));
  connect(sensors.bus, errors.avionicsBus) annotation(
      Line(points = {{-20, -60}, {2, -60}, {2, -44}, {20, -44}}, thickness = 0.5));
  connect(sensorsIdealCont.frame_a, lynxAirframe.ref_center) annotation(
      Line(points = {{-58, 88}, {-66, 88}, {-66, -12}, {10, -12}, {10, 10}, {0, 10}}, color = {95, 95, 95}));
  connect(sensorsIdealCont.bus.w_est, eulerRates.w) annotation(
      Line(points = {{-38, 88}, {-4, 88}, {-4, 58}, {6, 58}}, thickness = 0.5));
  connect(sensorsIdealCont.bus.q_est, eulerRates.q) annotation(
      Line(points = {{-38, 88}, {-4, 88}, {-4, 46}, {6, 46}}, thickness = 0.5));
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
    Components.Parts.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 2.268928027592628, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 1.48352986419518, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Components.Parts.LandDetector landDetector(groundlevel = 0)  annotation(
      Placement(visible = true, transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.LynxAirframe lynxAirframe annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
      Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4)  annotation(
      Placement(visible = true, transformation(origin = {32, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.SensorsIdealCont sensorsIdealCont annotation(
      Placement(visible = true, transformation(origin = {38, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(fixed.frame_b, launchRail.frame_a) annotation(
      Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
    connect(launchRail.frame_b_lug_bow, lynxAirframe.frame_a) annotation(
      Line(points = {{-40, 16}, {-20, 16}}));
    connect(launchRail.frame_b_lug_aft, lynxAirframe.frame_a1) annotation(
      Line(points = {{-40, 4}, {-20, 4}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, landDetector.frame_a) annotation(
      Line(points = {{0, 10}, {60, 10}, {60, -90}, {80, -90}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, aerodynamics.frame_b) annotation(
      Line(points = {{0, 10}, {60, 10}, {60, 30}, {80, 30}}));
  connect(vectorConstant.v, aerodynamics.finDeflection) annotation(
      Line(points = {{43, -16}, {80, -16}, {80, 24}}, color = {0, 0, 127}, thickness = 0.5));
  connect(lynxAirframe.ref_center, sensorsIdealCont.frame_a) annotation(
      Line(points = {{0, 10}, {4, 10}, {4, -64}, {28, -64}}, color = {95, 95, 95}));
    annotation(
      experiment(StartTime = 0, StopTime = 150, Tolerance = 0.0001, Interval = 0.015),
      Icon(graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}),
  Diagram(coordinateSystem(extent = {{-120, 40}, {120, -120}})));
  end LynxWithCanards;
  
  model LynxWithCanardsRoccaraso
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
    Components.Parts.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 0.1745329251994329, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 1.047197551196598, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Components.Parts.LandDetector landDetector annotation(
      Placement(visible = true, transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.LynxAirframe lynxAirframe annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Rockets.Lynx.Sensors sensors annotation(
      Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
      Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Rockets.Lynx.ControllersGlide controllers annotation(
      Placement(visible = true, transformation(origin = {18, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
      Line(points = {{0, 10}, {10, 10}, {10, -40}, {-60, -40}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, aerodynamics.frame_b) annotation(
      Line(points = {{0, 10}, {60, 10}, {60, 30}, {80, 30}}));
  connect(controllers.fin, aerodynamics.finDeflection) annotation(
      Line(points = {{30, -72}, {80, -72}, {80, 24}}, color = {0, 0, 127}, thickness = 0.5));
  connect(sensors.avionicsBus, controllers.avionicsBus) annotation(
      Line(points = {{-40, -50}, {28, -50}, {28, -64}}));
    annotation(
      experiment(StartTime = 0, StopTime = 150, Tolerance = 0.0001, Interval = 0.015),
      Icon(graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}));
  end LynxWithCanardsRoccaraso;
  
  model LynxLQ
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
    inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1.2}) annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Components.Parts.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 2.268928027592628, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 1.466076571675237, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Components.Parts.LandDetector landDetector(groundlevel = 0)  annotation(
      Placement(visible = true, transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.LynxAirframe lynxAirframe annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
      Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.ControllersLQIRates controllersLQ annotation(
      Placement(visible = true, transformation(origin = {-4, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.Atmosphere atmosphere(wind_speed = {0, 0, 0})  annotation(
      Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.SensorsIdealCont sensorsIdealCont annotation(
      Placement(visible = true, transformation(origin = {-20, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = time > 1)  annotation(
      Placement(visible = true, transformation(origin = {-78, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Blocks.Track track annotation(
      Placement(visible = true, transformation(origin = {30, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Blocks.GlideAngle glideAngle annotation(
      Placement(visible = true, transformation(origin = {30, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Blocks.Downrange downrange annotation(
      Placement(visible = true, transformation(origin = {30, 106}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Blocks.EulerRates eulerRates annotation(
      Placement(visible = true, transformation(origin = {-36, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(fixed.frame_b, launchRail.frame_a) annotation(
      Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
    connect(launchRail.frame_b_lug_bow, lynxAirframe.frame_a) annotation(
      Line(points = {{-40, 16}, {-20, 16}}));
    connect(launchRail.frame_b_lug_aft, lynxAirframe.frame_a1) annotation(
      Line(points = {{-40, 4}, {-20, 4}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, landDetector.frame_a) annotation(
      Line(points = {{0, 10}, {60, 10}, {60, -90}, {80, -90}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, aerodynamics.frame_b) annotation(
      Line(points = {{0, 10}, {60, 10}, {60, 30}, {80, 30}}));
  connect(sensorsIdealCont.bus, controllersLQ.avionicsBus) annotation(
      Line(points = {{-10, -36}, {20, -36}, {20, -60}, {6, -60}}, thickness = 0.5));
  connect(controllersLQ.fin, aerodynamics.finDeflection) annotation(
      Line(points = {{7, -68}, {72, -68}, {72, 24}, {80, 24}}, color = {0, 0, 127}, thickness = 0.5));
  connect(lynxAirframe.ref_center, sensorsIdealCont.frame_a) annotation(
      Line(points = {{0, 10}, {14, 10}, {14, -24}, {-50, -24}, {-50, -36}, {-30, -36}}, color = {95, 95, 95}));
  connect(booleanExpression.y, sensorsIdealCont.bus.liftoff) annotation(
      Line(points = {{-67, -20}, {-8, -20}, {-8, -36}, {-10, -36}}, color = {255, 0, 255}));
  connect(sensorsIdealCont.bus.v_est, glideAngle.v) annotation(
      Line(points = {{-10, -36}, {10, -36}, {10, 42}, {18, 42}}, thickness = 0.5));
  connect(sensorsIdealCont.bus.v_est, track.v) annotation(
      Line(points = {{-10, -36}, {10, -36}, {10, 74}, {18, 74}}, thickness = 0.5));
  connect(downrange.x, sensorsIdealCont.bus.x_est) annotation(
      Line(points = {{18, 106}, {10, 106}, {10, -36}, {-10, -36}}, color = {0, 0, 127}, thickness = 0.5));
  connect(eulerRates.w, sensorsIdealCont.bus.w_est) annotation(
      Line(points = {{-48, 72}, {-60, 72}, {-60, -56}, {-10, -56}, {-10, -36}}, color = {0, 0, 127}, thickness = 0.5));
  connect(eulerRates.q, sensorsIdealCont.bus.q_est) annotation(
      Line(points = {{-48, 60}, {-60, 60}, {-60, -56}, {-10, -56}, {-10, -36}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      experiment(StartTime = 0, StopTime = 60, Tolerance = 0.0001, Interval = 0.01),
      Icon(graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}));
  end LynxLQ;
equation

annotation(
    Icon(coordinateSystem(grid = {2, 0}), graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}));
end Simulations;
