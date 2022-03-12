within RocketControl.Rockets.Lynx.GNC;

model Errors
outer RocketControl.World.SimOptions opt;
  RocketControl.Interfaces.AvionicsBus bus_true annotation(
    Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.Angle v_ref_angle annotation(
    Placement(visible = true, transformation(origin = {30, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Blocks.Math.Vectors.Angle b_q_angle annotation(
    Placement(visible = true, transformation(origin = {70, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Clocked.VectorSample vectorSample1(n = 3) annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Clocked.VectorSample vectorSample(n = 4) annotation(
    Placement(visible = true, transformation(origin = {-50, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Blocks.Math.Quaternions.AngleError angleError annotation(
    Placement(visible = true, transformation(origin = {10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Clocked.VectorSample vectorSample2(n = 3) annotation(
    Placement(visible = true, transformation(origin = {-52, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.RollErrors rollErrors(target_heading = opt.roll_target_heading)  annotation(
    Placement(visible = true, transformation(origin = {30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus.v_ref, v_ref_angle.v2) annotation(
    Line(points = {{-100, -60}, {-6, -60}, {-6, -28}, {18, -28}}, thickness = 0.5));
  connect(bus_true.b_meas, vectorSample1.u) annotation(
    Line(points = {{-100, 60}, {-84, 60}, {-84, 90}, {-62, 90}}, thickness = 0.5));
  connect(vectorSample1.y, b_q_angle.v1) annotation(
    Line(points = {{-38, 90}, {58, 90}, {58, 94}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus_true.q_est, vectorSample.u) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 68}, {-62, 68}}, thickness = 0.5));
  connect(vectorSample.y, angleError.q1) annotation(
    Line(points = {{-38, 68}, {-24, 68}, {-24, 74}, {-2, 74}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.q_est, angleError.q2) annotation(
    Line(points = {{-100, -60}, {-40, -60}, {-40, 66}, {-2, 66}}, thickness = 0.5));
  connect(angleError.e, b_q_angle.v2) annotation(
    Line(points = {{22, 64}, {38, 64}, {38, 86}, {58, 86}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus_true.v_est, vectorSample2.u) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 16}, {-64, 16}}, thickness = 0.5));
  connect(vectorSample2.y, v_ref_angle.v1) annotation(
    Line(points = {{-40, 16}, {-6, 16}, {-6, -20}, {18, -20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(rollErrors.bus, bus_true) annotation(
    Line(points = {{20, 30}, {-76, 30}, {-76, 60}, {-100, 60}}, color = {255, 204, 51}, thickness = 0.5));
  annotation(
    Icon(graphics = {Rectangle(lineColor = {255, 179, 135}, fillColor = {255, 240, 158}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, 100}, {100, -100}}), Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {1, 11}, extent = {{-75, 75}, {75, -75}}, textString = "e"), Text(origin = {-1, -130}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-161, 30}, {161, -30}}, textString = "%name")}));
end Errors;
