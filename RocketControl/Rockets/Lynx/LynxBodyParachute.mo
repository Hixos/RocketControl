within RocketControl.Rockets.Lynx;

model LynxBodyParachute
  parameter SI.Time start_delay = 0.5;
  extends Rockets.Internal.PartialRocketBody;
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(animation = false, r = {-0.02, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(animation = false, r = {-0.43, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzleTranslation(animation = false, r = {-1.150, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.06, I_21 = 0, I_22 = 6.436, I_31 = 0, I_32 = 0, I_33 = 6.437, animation = false, enforceStates = false, m = 22, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  RocketControl.Components.Propulsion.M2000R m2000r(start_delay = start_delay) annotation(
    Placement(visible = true, transformation(origin = {30, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(rocket.frame_a, nozzleTranslation.frame_b) annotation(
    Line(points = {{30, 60}, {30, -20}}));
  connect(rocket.frame_a, ref_center) annotation(
    Line(points = {{30, 60}, {30, 0}, {100, 0}}));
  connect(lug_bow.frame_a, rocket.frame_a) annotation(
    Line(points = {{-40, 40}, {0, 40}, {0, 22}, {30, 22}, {30, 60}}, color = {95, 95, 95}));
  connect(lug_aft.frame_a, rocket.frame_a) annotation(
    Line(points = {{-40, -40}, {0, -40}, {0, 22}, {30, 22}, {30, 60}}, color = {95, 95, 95}));
  connect(frame_lug_bow, lug_bow.frame_b) annotation(
    Line(points = {{-100, 60}, {-68, 60}, {-68, 40}, {-60, 40}}));
  connect(lug_aft.frame_b, frame_lug_aft) annotation(
    Line(points = {{-60, -40}, {-100, -40}}, color = {95, 95, 95}));
  connect(nozzleTranslation.frame_a, m2000r.frame_b) annotation(
    Line(points = {{30, -40}, {30, -64}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Text(origin = {0, -220}, lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name")}),
    Diagram);
end LynxBodyParachute;
