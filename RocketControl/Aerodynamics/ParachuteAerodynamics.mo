within RocketControl.Aerodynamics;

model ParachuteAerodynamics
  outer RocketControl.World.Atmosphere atmosphere;
  outer RocketControl.World.Interfaces.WorldBase world;
  parameter SI.Area max_area;
  parameter SI.Area min_area;
  parameter SI.Duration opening_transient_duration;
  parameter Real Cd = 1;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  
  Modelica.Blocks.Interfaces.BooleanInput extended annotation(
    Placement(visible = true, transformation(origin = {50, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {42, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.BooleanInput deployed annotation(
    Placement(visible = true, transformation(origin = {-52, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    
    SI.Density rho;
  SI.Time t_0;
  SI.Area S;
  SI.Force drag[3];
  SI.Force drag_norm;
  RocketControl.Interfaces.AeroStateInput aeroStateInput annotation(
    Placement(visible = true, transformation(origin = {-102, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-102, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
initial equation
  t_0 = 0;
equation
  when extended then
    t_0 = pre(time);
  end when;
  
  if extended and time > t_0 + opening_transient_duration then
    S = max_area;
  elseif extended then
    S = (time - t_0)/opening_transient_duration*(max_area - min_area) + min_area;
   elseif deployed then
   S = min_area;
   else
   S = 0;
  end if;
  
  drag = -0.5*rho*norm(aeroStateInput.v)*S*Cd*aeroStateInput.v;
  drag_norm = norm(drag);
  frame_b.f = -drag;
  rho = atmosphere.density(world.altitude(frame_b.r_0));
  frame_b.t = zeros(3);
  annotation(
    Icon(graphics = {Polygon(origin = {10, 11}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Sphere, points = {{-40, 21}, {-12, 11}, {10, -15}, {16, -45}, {34, -29}, {38, -3}, {32, 17}, {20, 29}, {-2, 37}, {-22, 31}, {-40, 21}}), Ellipse(origin = {-65, -59}, fillPattern = FillPattern.Solid, extent = {{-7, 7}, {7, -7}}), Line(origin = {-33.7615, -12.6624}, points = {{3.76146, 44.6624}, {-32.2385, -45.3376}, {31.7615, 34.6624}}), Line(origin = {-20.6532, -31.7916}, points = {{40.6532, 27.7916}, {-47.3468, -28.2084}, {46.6532, -2.20836}}), Line(origin = {66, 65}, points = {{-24, -23}, {24, 23}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Text(origin = {83, 42}, extent = {{-25, 26}, {25, -26}}, textString = "D"), Text(origin = {4, -186}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
end ParachuteAerodynamics;
