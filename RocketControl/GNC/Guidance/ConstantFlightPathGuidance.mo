within RocketControl.GNC.Guidance;

model ConstantFlightPathGuidance
  outer RocketControl.World.Atmosphere atmosphere;

  parameter Real k = 0.7;
  parameter Real kint = 1;
  parameter Real int_lim = 10;
  parameter SI.Angle heading(displayUnit = "deg") = 0;
  parameter SI.Angle flightpathangle(displayUnit = "deg") = from_deg(84);
  final parameter Real V_dir_target_ned[3] = {cos(heading) * cos(flightpathangle), sin(heading) * cos(flightpathangle), -sin(flightpathangle)};
  
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput acc_err_int[3](start = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SI.Velocity V_body[3];
  SI.Velocity V_target_body[3];
  SI.Velocity V_err_body[3];
  SI.Acceleration acc_target[2];
  SI.Acceleration acc_target_sat[2];
  SI.Acceleration acc_meas[2];
  SI.Acceleration acc_err[3];
  SI.Acceleration acc_max;
  SI.Acceleration g[3];
equation
  g = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, {0, 0, 9.80665});
  V_body = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, bus.v_est);
  V_target_body = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, V_dir_target_ned) * norm(bus.v_est);
  V_err_body = V_target_body - V_body;
  acc_target = k * V_err_body[2:3];
  acc_max = 0.5 * atmosphere.density(-bus.x_est[3]) * norm(bus.v_est) ^ 2 * (3.14 * 0.15 ^ 2 / 4) * 24 * from_deg(4) / 22;
  if noEvent(acc_max < norm(acc_target)) then
    acc_target_sat = acc_target_sat / norm(acc_target) * acc_max;
  else
    acc_target_sat = acc_target;
  end if;
  acc_meas = bus.a_meas[2:3] + g[2:3];
  acc_err = cat(1, {0}, acc_target_sat - acc_meas);
  if noEvent(bus.control_enable) then
    for i in 1:3 loop
      if abs(acc_err_int[i]) > int_lim and acc_err[i] * acc_err[i] > 0 then
        der(acc_err_int[i]) = 0;
      else
        der(acc_err_int[i]) = - kint * acc_err[i];
      end if;
    end for;
  else
    der(acc_err_int) = {0, 0, 0};
  end if;
  annotation(
    Icon(graphics = {Rectangle(fillColor = {240, 255, 244}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 12), Line(origin = {20.87, 26.51}, points = {{-47, -57}, {47, 57}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Polygon(origin = {-54, -64}, fillColor = {156, 212, 255}, fillPattern = FillPattern.VerticalCylinder, points = {{10, 18}, {20, 22}, {16, 12}, {-4, -8}, {-6, -18}, {-20, -4}, {-10, -2}, {10, 18}}), Text(origin = {102, -31}, lineColor = {102, 102, 102}, extent = {{-44, 17}, {44, -17}}, textString = "v_err"), Text(origin = {-26, 41}, extent = {{-44, 17}, {44, -17}}, textString = "v = k"), Text(origin = {2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end ConstantFlightPathGuidance;
