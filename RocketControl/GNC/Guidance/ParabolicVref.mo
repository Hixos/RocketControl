within RocketControl.GNC.Guidance;

model ParabolicVref
extends Icon;
  parameter Modelica.Units.SI.Height target_apogee = 3000;
  parameter SI.Angle flightpathangle_0(displayUnit = "deg") = from_deg(84);
  parameter SI.Angle heading(displayUnit = "deg") = 0;
  parameter SI.Time t0 = 0.5;
  
  final parameter Real c1 = sqrt(2*9.80665*target_apogee);
  final parameter SI.Velocity vx_parab =  sqrt(2*9.80665*target_apogee)/tan(flightpathangle_0);


  Modelica.Blocks.Interfaces.RealOutput V_ref[3](start = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    SI.Angle flightpathangle(displayUnit = "deg");
    
    Real z_ref;
equation
  z_ref = -0.5*9.80665*(time-t0)^2+c1*(time-t0);
  flightpathangle = atan((-9.80665*(time - t0)+c1)/vx_parab);
  V_ref = {cos(heading) * cos(flightpathangle), sin(heading) * cos(flightpathangle), -sin(flightpathangle)};
annotation(
    Icon(graphics = {Line(origin = {2.01, 7.92}, points = {{-76.0134, -67.9186}, {-68.0134, -23.9186}, {-52.0134, 24.0814}, {-32.0134, 54.0814}, {-4.01343, 68.0814}, {3.98657, 68.0814}, {33.9866, 54.0814}, {51.9866, 24.0814}, {65.9866, -19.9186}, {75.9866, -65.9186}}, color = {255, 0, 0}, arrowSize = 5), Line(origin = {2, 9}, points = {{0, 67}, {0, -67}}, arrow = {Arrow.Filled, Arrow.None}), Line(origin = {-54, 30}, points = {{-8, -32}, {8, 32}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 5), Rectangle(origin = {3, -61}, fillPattern = FillPattern.Solid, extent = {{-91, 3}, {91, -3}})}));
end ParabolicVref;
