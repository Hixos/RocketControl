within RocketControl.GNC.Guidance;

model ConstantFlightPathGuidanceDiscrete2
  outer RocketControl.World.Atmosphere atmosphere;
  import Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2;
  import Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve1;
  
  parameter Real k = 0.7;
  parameter Real kint = 1;
  parameter Real int_lim = 10;
  parameter Real dt;
  
  parameter SI.Angle heading(displayUnit = "deg") = 0;
  parameter SI.Angle flightpathangle(displayUnit = "deg") = from_deg(84);
  final parameter Real V_dir_target_ned[3] = {cos(heading) * cos(flightpathangle), sin(heading) * cos(flightpathangle), -sin(flightpathangle)};
  
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput a_err_int_b[3](start = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Real x_w;
  
  SI.Velocity V_target[3];
  SI.Velocity V_err[3];
  
  SI.Acceleration a_target[3];
  SI.Acceleration a_target_sat[3];
  SI.Acceleration a_meas[3];
  
  SI.Acceleration a_err[3];
  SI.Acceleration a_err_lat[3];
  
  SI.Acceleration a_max;
  SI.Acceleration g[3];
  
  SI.Velocity a_err_int[3](start = {0,0,0});
  SI.Velocity a_err_int_prev[3];
  
  
equation
  x_w = resolve1(bus.q_est, {1,0,0});
  
  a_max = 0.5 * atmosphere.density(-bus.x_est[3]) * norm(bus.v_est) ^ 2 * (3.14 * 0.15 ^ 2 / 4) * 24 * from_deg(7) / 22;
  
  g = {0, 0, 9.80665};
  
  V_target = V_dir_target_ned * norm(bus.v_est); 
  V_err = V_target - bus.v_est;
  
  a_target = k * V_err;  
  
  if noEvent(norm(a_target) > a_max) then
    a_target_sat = a_target / norm(a_target) * a_max;
  else
    a_target_sat = a_target;
  end if;
  
  a_meas = resolve1(bus.q_est, bus.a_meas) + g;
  
  a_err = a_target_sat - a_meas;
  
  a_err_lat = a_err - (a_err*x_w)*x_w; // Remove component along body x axis
  
  a_err_int_prev = previous(a_err_int);
  
  if noEvent(bus.control_enable) then
    for i in 1:3 loop
      if abs(acc_err_int_prev[i]) > int_lim and acc_err[i] * acc_err_int_prev[i] < 0 then
        acc_err_int[i] = acc_err_int_prev[i];
      else
        acc_err_int[i] = acc_err_int_prev[i] - kint * acc_err[i] * dt;
      end if;
    end for;
  else
    acc_err_int = acc_err_int_prev;
  end if;
  
  a_err_int_b = resolve2(bus.q_est, a_err_int);
  annotation(
    Icon(graphics = {Rectangle(fillColor = {240, 255, 244}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 12), Line(origin = {20.87, 26.51}, points = {{-47, -57}, {47, 57}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Polygon(origin = {-54, -64}, fillColor = {156, 212, 255}, fillPattern = FillPattern.VerticalCylinder, points = {{10, 18}, {20, 22}, {16, 12}, {-4, -8}, {-6, -18}, {-20, -4}, {-10, -2}, {10, 18}}), Text(origin = {102, -31}, lineColor = {102, 102, 102}, extent = {{-44, 17}, {44, -17}}, textString = "v_err"), Text(origin = {-26, 41}, extent = {{-44, 17}, {44, -17}}, textString = "v = k"), Text(origin = {2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end ConstantFlightPathGuidanceDiscrete2;
