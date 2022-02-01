within RocketControl.GNC.Guidance;

model VelocityRef
  extends Icon;
  parameter SI.Angle track_ref(displayUnit = "deg");
  parameter SI.Angle climbangle_max(displayUnit = "deg") = from_deg(84);
  Modelica.Blocks.Interfaces.RealOutput v_ref[3](each final unit = "m/s", each final quantity = "Velocity") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SI.Velocity v_horiz;
  SI.Velocity v_horiz_min;
  SI.Velocity v_horiz_ref[3];
protected
  final parameter Real z[3] = {0, 0, 1};
  final parameter Real dir_horiz[3] = {cos(track_ref), sin(track_ref), 0};
equation
  v_horiz = norm(bus.v_est - bus.v_est * z * z);
  v_horiz_min = norm(bus.v_est) * cos(climbangle_max);
  v_horiz_ref = max(v_horiz, v_horiz_min) * dir_horiz;
  if norm(bus.v_est) > 1 then
    v_ref = v_horiz_ref + sqrt(norm(bus.v_est) ^ 2 - norm(v_horiz_ref) ^ 2) * z * sign(bus.v_est * z);
  else
    v_ref = bus.v_est;
  end if;
  annotation(
    Icon(graphics = {Text(origin = {-4, 11}, extent = {{-82, 55}, {82, -55}}, textString = "v_ref")}));
end VelocityRef;
