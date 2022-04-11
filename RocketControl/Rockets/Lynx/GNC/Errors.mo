within RocketControl.Rockets.Lynx.GNC;

model Errors
outer RocketControl.World.SimOptions opt;
  RocketControl.Interfaces.AvionicsBus bus_true annotation(
    Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.Angle v_ref_angle annotation(
    Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Blocks.Math.Vectors.Angle b_q_angle annotation(
    Placement(visible = true, transformation(origin = {70, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Clocked.VectorSample vectorSample1(n = 3) annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Clocked.VectorSample vectorSample(n = 4) annotation(
    Placement(visible = true, transformation(origin = {-50, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Blocks.Math.Quaternions.AngleError angleError annotation(
    Placement(visible = true, transformation(origin = {10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.RollErrors rollErrors(target_heading = opt.roll_target_heading)  annotation(
    Placement(visible = true, transformation(origin = {30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorAdd pos_err(gain = {-1, 1})  annotation(
    Placement(visible = true, transformation(origin = {-50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorNorm pos_err_norm annotation(
    Placement(visible = true, transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorAdd vel_err(gain = {1, -1}) annotation(
    Placement(visible = true, transformation(origin = {-50, -96}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorNorm vel_err_norm annotation(
    Placement(visible = true, transformation(origin = {-10, -96}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Clocked.VectorSample vectorSample3(n = 3) annotation(
    Placement(visible = true, transformation(origin = {-76, -16}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  RocketControl.Components.Clocked.VectorSample vectorSample4(n = 3) annotation(
    Placement(visible = true, transformation(origin = {-96, -16}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  RocketControl.Blocks.Math.Vectors.Angle b_q_angle_true annotation(
    Placement(visible = true, transformation(origin = {72, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Real e[3];
Real xi;
Real theta;
  Components.Clocked.VectorSample vectorSample2(n = 3) annotation(
    Placement(visible = true, transformation(origin = {-52, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  theta = angleError.angle_err;
  xi = b_q_angle_true.angle;
  e = angleError.e;
  connect(bus.v_ref, v_ref_angle.v2) annotation(
    Line(points = {{-100, -60}, {-6, -60}, {-6, -4}, {18, -4}}, thickness = 0.5));
  connect(bus_true.q_est, vectorSample.u) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 68}, {-62, 68}}, thickness = 0.5));
  connect(vectorSample.y, angleError.q1) annotation(
    Line(points = {{-38, 68}, {-24, 68}, {-24, 74}, {-2, 74}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.q_est, angleError.q2) annotation(
    Line(points = {{-100, -60}, {-40, -60}, {-40, 66}, {-2, 66}}, thickness = 0.5));
  connect(angleError.e, b_q_angle.v2) annotation(
    Line(points = {{22, 64}, {38, 64}, {38, 86}, {58, 86}}, color = {0, 0, 127}, thickness = 0.5));
  connect(rollErrors.bus, bus_true) annotation(
    Line(points = {{20, 30}, {-76, 30}, {-76, 60}, {-100, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(bus.x_est, pos_err.v2) annotation(
    Line(points = {{-100, -60}, {-76, -60}, {-76, -74}, {-62, -74}}, thickness = 0.5));
  connect(pos_err.vc, pos_err_norm.v) annotation(
    Line(points = {{-39, -70}, {-22, -70}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vel_err.vc, vel_err_norm.v) annotation(
    Line(points = {{-38, -96}, {-22, -96}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.v_est, vel_err.v2) annotation(
    Line(points = {{-100, -60}, {-76, -60}, {-76, -100}, {-62, -100}}, thickness = 0.5));
  connect(bus_true.x_est, vectorSample3.u) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, -4}}, thickness = 0.5));
  connect(vectorSample3.y, pos_err.v1) annotation(
    Line(points = {{-76, -26}, {-76, -66}, {-62, -66}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus_true.v_est, vectorSample4.u) annotation(
    Line(points = {{-100, 60}, {-96, 60}, {-96, -4}}, thickness = 0.5));
  connect(vectorSample4.y, vel_err.v1) annotation(
    Line(points = {{-96, -26}, {-98, -26}, {-98, -92}, {-62, -92}}, color = {0, 0, 127}, thickness = 0.5));
  connect(b_q_angle.v1, bus.b_meas) annotation(
    Line(points = {{58, 94}, {-100, 94}, {-100, -60}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorSample1.y, b_q_angle_true.v1) annotation(
    Line(points = {{-38, 90}, {60, 90}, {60, 62}}, color = {0, 0, 127}, thickness = 0.5));
  connect(angleError.e, b_q_angle_true.v2) annotation(
    Line(points = {{22, 64}, {38, 64}, {38, 54}, {60, 54}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus_true.b_meas, vectorSample1.u) annotation(
    Line(points = {{-100, 60}, {-78, 60}, {-78, 90}, {-62, 90}}, thickness = 0.5));
  connect(bus_true.v_est, vectorSample2.u) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 16}, {-64, 16}}, thickness = 0.5));
  connect(bus.v_est, v_ref_angle.v1) annotation(
    Line(points = {{-100, -60}, {-24, -60}, {-24, 4}, {18, 4}}, thickness = 0.5));
  annotation(
    Icon(graphics = {Rectangle(lineColor = {255, 179, 135}, fillColor = {255, 240, 158}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, 100}, {100, -100}}), Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {1, 11}, extent = {{-75, 75}, {75, -75}}, textString = "e"), Text(origin = {-1, -130}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-161, 30}, {161, -30}}, textString = "%name")}));
end Errors;
