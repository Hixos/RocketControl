within RocketControl.Aerodynamics;

model AeroAnglesSensor
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  import Modelica.Mechanics.MultiBody.Frames;
  import Modelica.Math.Vectors;
  outer World.Atmosphere atmosphere;
  outer World.MyWorld world;
  parameter Boolean limit_alpha_dot = true;
  parameter SI.Angle alpha_dot_max = from_deg(360 * 5);
  Modelica.Blocks.Interfaces.RealOutput alpha(final quantity = "Angle", final unit = "rad", displayUnit = "deg") annotation(
    Placement(visible = true, transformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SI.Velocity v_b[3] "Velocity relative to wind in body frame (body frame)";
  SI.Velocity v_w[3] "Wind speed at current position";
  SI.Velocity v_norm;
  parameter SI.Velocity v_small = 1e-5 "Prevent division by zero when velocity is too small";
  Modelica.Blocks.Interfaces.RealOutput beta(final quantity = "Angle", final unit = "rad", displayUnit = "deg") annotation(
    Placement(visible = true, transformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput alpha_dot(displayUnit = "deg", quantity = "Angle", unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  v_w = atmosphere.windSpeed(world.altitude(frame_a.r_0));
  v_b = Frames.resolve2(frame_a.R, der(frame_a.r_0) - v_w);
  v_norm = Vectors.norm(v_b);
  if noEvent(abs(v_b[1]) > v_small) then
    alpha = atan(v_b[3] / v_b[1]);
  elseif noEvent(abs(v_b[3]) > abs(v_b[1])) then
    alpha = sign(v_b[3]) * pi / 2;
  else
    alpha = 0;
  end if;
  if noEvent(abs(der(alpha)) > alpha_dot_max and limit_alpha_dot) then
    alpha_dot = alpha_dot_max * sign(der(alpha));
  else
    alpha_dot = der(alpha);
  end if;
  if noEvent(abs(v_b[1]) > v_small) then
    beta = atan(v_b[2] / v_b[1]);
//sideslip = asin(v_b[2] / v_norm);
  elseif noEvent(abs(v_b[2]) > v_small) then
    beta = Modelica.Units.Conversions.from_deg(90 * sign(v_b[2]));
  else
    beta = 0;
  end if;
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Text(origin = {113, 44}, extent = {{-47, 16}, {47, -16}}, textString = "alpha"), Text(origin = {111, -95}, extent = {{-57, 15}, {57, -15}}, textString = "beta"), Text(origin = {118, -25}, extent = {{-60, 15}, {60, -15}}, textString = "alpha_dot")}));
end AeroAnglesSensor;
