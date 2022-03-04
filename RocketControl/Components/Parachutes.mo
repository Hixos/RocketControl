within RocketControl.Components;

package Parachutes
  model SimpleParachute
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a; 
  extends RocketControl.Interfaces.PartialConditionalEnablePort(useEnablePort = true);
    parameter SI.Area surface;
    parameter SI.Area initial_surface;
    parameter SI.Duration opening_delay;
    parameter SI.Duration opening_transient_duration;
    parameter Real Cd;
    parameter Real CLL_p;
    parameter SI.Length d;
  
  RocketControl.Aerodynamics.ParachuteAerodynamics parachuteAerodynamics(CLL_p = CLL_p,Cd = Cd, d = d, max_area = surface, min_area = initial_surface, opening_transient_duration = opening_transient_duration) annotation(
      Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.AeroStateSensor aeroStateSensor annotation(
      Placement(visible = true, transformation(origin = {-30, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.MathBoolean.OnDelay onDelay(delayTime = opening_delay)  annotation(
      Placement(visible = true, transformation(origin = {20, 48}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
  equation

    connect(frame_a, parachuteAerodynamics.frame_b) annotation(
      Line(points = {{-100, 0}, {0, 0}}));
  connect(aeroStateSensor.frame_a, frame_a) annotation(
      Line(points = {{-40, -40}, {-60, -40}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(aeroStateSensor.aeroStateOutput, parachuteAerodynamics.aeroStateInput) annotation(
      Line(points = {{-20, -40}, {-12, -40}, {-12, -6}, {0, -6}}));
  connect(enable, parachuteAerodynamics.deployed) annotation(
      Line(points = {{0, 106}, {4, 106}, {4, 10}}, color = {255, 0, 255}));
  connect(enable, onDelay.u) annotation(
      Line(points = {{0, 106}, {2, 106}, {2, 64}, {20, 64}, {20, 54}}, color = {255, 0, 255}));
  connect(onDelay.y, parachuteAerodynamics.extended) annotation(
      Line(points = {{20, 44}, {14, 44}, {14, 10}}, color = {255, 0, 255}));
  annotation(
      Icon(graphics = {Polygon(origin = {32, 43}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Sphere, points = {{-82, 31}, {-36, 17}, {-2, -13}, {18, -55}, {22, -101}, {60, -55}, {68, -3}, {46, 37}, {6, 55}, {-36, 55}, {-82, 31}}), Line(origin = {-100.514, -2.19848}, points = {{51.7615, 76.6624}, {9.7615, -73.3376}, {97.761, 62.6624}}), Line(origin = {-41.9419, -46.9462}, points = {{92.6532, 35.7916}, {-47.3468, -28.2084}, {96.6532, -10.2084}}), Text(origin = {-100, -36}, lineColor = {102, 102, 102}, extent = {{-56, 20}, {56, -20}}, textString = "link"), Ellipse(origin = {-89, -75}, fillPattern = FillPattern.Solid, extent = {{-7, 7}, {7, -7}}), Text(origin = {0, 30}, lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name")}));end SimpleParachute;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Parachutes;
