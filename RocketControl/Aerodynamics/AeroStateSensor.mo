within RocketControl.Aerodynamics;

model AeroStateSensor
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  import Modelica.Mechanics.MultiBody.Frames;
  import Modelica.Math.Vectors;
  RocketControl.Aerodynamics.AeroAnglesSensor aeroAnglesSensor annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  outer World.Atmosphere atmosphere;
  outer World.Interfaces.WorldBase world;
  Modelica.Units.SI.Angle beta2;
  SI.Velocity[3] v_w;
  SI.Velocity[3] v;
  Aerodynamics.Interfaces.AeroStateOutput aeroStateOutput annotation(
    Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  v_w = atmosphere.windSpeed(world.altitude(frame_a.r_0));
  aeroStateOutput.alpha = aeroAnglesSensor.alpha;
  aeroStateOutput.alpha_dot = aeroAnglesSensor.alpha_dot;
  aeroStateOutput.beta = aeroAnglesSensor.beta;
  aeroStateOutput.mach = norm(v) / atmosphere.speedOfSound(world.altitude(frame_a.r_0));
  aeroStateOutput.altitude = world.altitude(frame_a.r_0);
  v = Frames.resolve2(frame_a.R, der(frame_a.r_0) - v_w);
  aeroStateOutput.v = v;
  aeroStateOutput.w = absoluteAngularVelocity.w;
  beta2 = atan2(v[2], v[1]);
  connect(aeroAnglesSensor.frame_a, frame_a) annotation(
    Line(points = {{-10, 40}, {-55, 40}, {-55, 0}, {-100, 0}}));
  connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
    Line(points = {{-100, 0}, {-54, 0}, {-54, -40}, {-10, -40}}));
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Text(origin = {1, -40}, lineColor = {64, 64, 64}, extent = {{-101, 20}, {101, -20}}, textString = "aero"), Line(origin = {84.1708, -2.17082}, points = {{-14.1708, 2.17082}, {11.8292, 2.17082}, {13.8292, -1.82918}})}));
end AeroStateSensor;
