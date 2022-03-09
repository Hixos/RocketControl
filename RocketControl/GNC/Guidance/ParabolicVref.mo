within RocketControl.GNC.Guidance;

model ParabolicVref
extends Icon;
  parameter Modelica.Units.SI.Height target_apogee = 3000;
  parameter SI.Angle flightpathangle_0(displayUnit = "deg") = from_deg(84);
  parameter SI.Angle heading(displayUnit = "deg") = 0;
  
  
  final parameter Real c1 = sqrt(2*9.80665*target_apogee);
  final parameter SI.Velocity vx_parab =  sqrt(2*9.80665*target_apogee)/tan(flightpathangle_0);


  Modelica.Blocks.Interfaces.RealOutput V_ref[3](start = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    SI.Angle flightpathangle(displayUnit = "deg");
    
    Real z_ref;
equation
  z_ref = -0.5*9.80665*(time-0.5)^2+c1*(time-0.5);
  flightpathangle = atan((-9.80665*(time - 0.5)+c1)/vx_parab);
  V_ref = {cos(heading) * cos(flightpathangle), sin(heading) * cos(flightpathangle), -sin(flightpathangle)};
annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end ParabolicVref;
