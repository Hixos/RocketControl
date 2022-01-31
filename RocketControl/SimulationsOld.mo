within RocketControl;

package SimulationsOld
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
    inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Parts.LandDetector landDetector annotation(
      Placement(visible = true, transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LynxRocket lynxRocket annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 2.268928027592628, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 1.466076571675237, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.Atmosphere atmosphere annotation(
      Placement(visible = true, transformation(origin = {-90, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.LynxIdealSensors lynxIdealSensors annotation(
      Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(fixed.frame_b, launchRail.frame_a) annotation(
      Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, lynxRocket.frame_lug_bow) annotation(
      Line(points = {{-40, 16}, {-20, 16}}));
  connect(launchRail.frame_b_lug_aft, lynxRocket.frame_lug_aft) annotation(
      Line(points = {{-40, 4}, {-20, 4}}));
  connect(lynxRocket.ref_center, landDetector.frame_a) annotation(
      Line(points = {{0, 10}, {62, 10}, {62, 90}, {80, 90}}));
  connect(lynxIdealSensors.frame_a, lynxRocket.ref_center) annotation(
      Line(points = {{20, -10}, {13, -10}, {13, 10}, {0, 10}}, color = {95, 95, 95}));
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
    Components.Parts.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 0, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 1.466076571675237, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.LynxAirframe lynxAirframe annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
      Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.ControllersLQ controllersLQ annotation(
      Placement(visible = true, transformation(origin = {-4, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.Atmosphere atmosphere(wind_speed = {10, 0, 0})  annotation(
      Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.SensorsIdealCont sensorsIdealCont annotation(
      Placement(visible = true, transformation(origin = {-18, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = time > 1 and time < 8)  annotation(
      Placement(visible = true, transformation(origin = {-78, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Blocks.Track track annotation(
      Placement(visible = true, transformation(origin = {30, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Blocks.GlideAngle glideAngle annotation(
      Placement(visible = true, transformation(origin = {30, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Blocks.Downrange downrange annotation(
      Placement(visible = true, transformation(origin = {30, 106}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Blocks.EulerRates eulerRates annotation(
      Placement(visible = true, transformation(origin = {-36, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.DescentDetector descentDetector annotation(
      Placement(visible = true, transformation(origin = {86, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.Deflection2Control deflection2Control annotation(
      Placement(visible = true, transformation(origin = {122, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.FinServoMotor finServoMotor(a = {0.07692, 1}, b = {0, 1})  annotation(
      Placement(visible = true, transformation(origin = {50, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(fixed.frame_b, launchRail.frame_a) annotation(
      Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
    connect(launchRail.frame_b_lug_bow, lynxAirframe.frame_a) annotation(
      Line(points = {{-40, 16}, {-20, 16}}));
    connect(launchRail.frame_b_lug_aft, lynxAirframe.frame_a1) annotation(
      Line(points = {{-40, 4}, {-20, 4}}, color = {95, 95, 95}));
    connect(lynxAirframe.ref_center, aerodynamics.frame_b) annotation(
      Line(points = {{0, 10}, {60, 10}, {60, 30}, {80, 30}}));
    connect(sensorsIdealCont.bus, controllersLQ.avionicsBus) annotation(
      Line(points = {{-8, -38}, {20, -38}, {20, -60}, {6, -60}}, thickness = 0.5));
    connect(lynxAirframe.ref_center, sensorsIdealCont.frame_a) annotation(
      Line(points = {{0, 10}, {14, 10}, {14, -24}, {-50, -24}, {-50, -38}, {-28, -38}}, color = {95, 95, 95}));
    connect(booleanExpression.y, sensorsIdealCont.bus.liftoff) annotation(
      Line(points = {{-67, -20}, {-8, -20}, {-8, -38}}, color = {255, 0, 255}));
    connect(sensorsIdealCont.bus.v_est, glideAngle.v) annotation(
      Line(points = {{-8, -38}, {10, -38}, {10, 42}, {18, 42}}, thickness = 0.5));
    connect(sensorsIdealCont.bus.v_est, track.v) annotation(
      Line(points = {{-8, -38}, {10, -38}, {10, 74}, {18, 74}}, thickness = 0.5));
    connect(downrange.x, sensorsIdealCont.bus.x_est) annotation(
      Line(points = {{18, 106}, {10, 106}, {10, -38}, {-8, -38}}, color = {0, 0, 127}, thickness = 0.5));
    connect(eulerRates.w, sensorsIdealCont.bus.w_est) annotation(
      Line(points = {{-48, 72}, {-60, 72}, {-60, -56}, {-8, -56}, {-8, -38}}, color = {0, 0, 127}, thickness = 0.5));
    connect(eulerRates.q, sensorsIdealCont.bus.q_est) annotation(
      Line(points = {{-48, 60}, {-60, 60}, {-60, -56}, {-8, -56}, {-8, -38}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      experiment(StartTime = 0, StopTime = 60, Tolerance = 0.0001, Interval = 0.01),
      Icon(graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}));
  end LynxLQ;

  model LynxRepack
  equation

    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end LynxRepack;
equation

annotation(
    Icon(coordinateSystem(grid = {2, 0}), graphics = {Polygon(fillColor = {29, 163, 125}, fillPattern = FillPattern.Solid, lineThickness = 1.5, points = {{-80, 80}, {-80, -80}, {80, 0}, {-80, 80}})}));
end SimulationsOld;
