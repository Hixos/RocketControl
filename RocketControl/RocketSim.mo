within RocketControl;

model RocketSim
  import Modelica.Units.Conversions.from_deg;

  parameter SI.Angle elevation(displayUnit = "deg") = from_deg(87);
  parameter SI.Angle azimuth(displayUnit = "deg") = from_deg(45);
  parameter SI.AngularVelocity w0[3] = from_deg({0, 0, 0});
  parameter SI.Velocity v0[3] = {0, 0, 0};
  
  Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.066590312, I_21 = -0.003190759, I_22 = 9.822815884, I_31 = 0.00128563, I_32 = -0.000234088, I_33 = 9.822815884, angles_fixed = true, angles_start(each displayUnit = "rad") = {azimuth, elevation, 0}, enforceStates = true, m = 22, r_0(each fixed = true), r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, sequence_start = {3, 2, 1}, v_0(each fixed = true, start = v0), w_0_fixed = false, w_a(each fixed = true, start = w0))  annotation(
    Placement(visible = true, transformation(origin = {0, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  RocketControl.Components.Motors.M2000R m2000r annotation(
    Placement(visible = true, transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzleTranslation(r = {-1.150, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  inner RocketControl.World.Atmosphere atmosphere annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.MyWorld world(n = {0, 0, 1})  annotation(
    Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(sequence = {3, 2, 1})  annotation(
    Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.Aerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {70, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  when rocket.r_0[3] > 0 and time > 1 then
    terminate("Simulation terminated successfully");
  end when;
  connect(m2000r.frame_b, nozzleTranslation.frame_a) annotation(
    Line(points = {{0, -40}, {0, -20}}, color = {95, 95, 95}));
  connect(nozzleTranslation.frame_b, rocket.frame_a) annotation(
    Line(points = {{0, 0}, {0, 32}}));
  connect(aerodynamics.frame_b, rocket.frame_a) annotation(
    Line(points = {{60, 40}, {40, 40}, {40, 20}, {0, 20}, {0, 32}}, color = {95, 95, 95}));
  connect(absoluteAngles.frame_a, rocket.frame_a) annotation(
    Line(points = {{60, 0}, {40, 0}, {40, 20}, {0, 20}, {0, 32}}, color = {95, 95, 95}));
end RocketSim;
