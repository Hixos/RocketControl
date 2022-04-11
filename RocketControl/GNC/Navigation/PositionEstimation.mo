within RocketControl.GNC.Navigation;

model PositionEstimation
  extends RocketControl.Icons.Navigation;
  parameter SI.Position sigma_pos;
  parameter SI.Position sigma_vel;
  parameter Real[6] sigma_gps;
  parameter Real[6] x0 = {0, 0, 0, 0, 0, 0};
  parameter Integer samplingPeriodMs;
  final parameter SI.Duration T = samplingPeriodMs / 1000;
  RocketControl.GNC.DiscreteKalmanFilter discreteKalmanFilter(A = [identity(3), identity(3) * T; zeros(3, 3), identity(3)], B = [zeros(3, 3); identity(3) * T], C = identity(6), Q = [identity(3) * sigma_pos ^ 2, zeros(3, 3); zeros(3, 3), identity(3) * sigma_vel ^ 2] .* T, R = diagonal(sigma_gps .^ 2), m = 3, n = 6, p = 6, x0 = x0) annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q[4] annotation(
    Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput acc_body[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 32}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 32}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput pos_ned[3] annotation(
    Placement(visible = true, transformation(origin = {-120, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -36}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput vel_ned[3] annotation(
    Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -98}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Blocks.Math.Quaternions.resolve1 acc_ned annotation(
    Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant gravity(k = {0, 0, Modelica.Constants.g_n}) annotation(
    Placement(visible = true, transformation(origin = {-32, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorAdd acc_inertial annotation(
    Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput pos_est[3] annotation(
    Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput vel_est[3] annotation(
    Placement(visible = true, transformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanConstant enable annotation(
    Placement(visible = true, transformation(origin = {26, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanInput gps_fix annotation(
    Placement(visible = true, transformation(origin = {50, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-40, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
equation
  connect(q, acc_ned.q) annotation(
    Line(points = {{-120, 80}, {-76, 80}, {-76, 56}, {-62, 56}}, color = {0, 0, 127}, thickness = 0.5));
  connect(acc_body, acc_ned.x2) annotation(
    Line(points = {{-120, 32}, {-76, 32}, {-76, 44}, {-62, 44}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gravity.v, acc_inertial.v1) annotation(
    Line(points = {{-20, 90}, {-12, 90}, {-12, 34}, {-2, 34}}, color = {0, 0, 127}, thickness = 0.5));
  connect(acc_ned.x1, acc_inertial.v2) annotation(
    Line(points = {{-38, 50}, {-22, 50}, {-22, 26}, {-2, 26}}, color = {0, 0, 127}, thickness = 0.5));
  connect(acc_inertial.vc, discreteKalmanFilter.u) annotation(
    Line(points = {{22, 30}, {32, 30}, {32, -6}, {40, -6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(discreteKalmanFilter.x_est[1:3], pos_est) annotation(
    Line(points = {{60, 6}, {74, 6}, {74, 40}, {110, 40}}, color = {0, 0, 127}, thickness = 0.5));
  connect(discreteKalmanFilter.x_est[4:6], vel_est) annotation(
    Line(points = {{60, -6}, {74, -6}, {74, -20}, {110, -20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(enable.y, discreteKalmanFilter.enable) annotation(
    Line(points = {{38, 78}, {50, 78}, {50, 10}}, color = {255, 0, 255}));
  connect(pos_ned, discreteKalmanFilter.y_meas[1:3]) annotation(
    Line(points = {{-120, -30}, {-20, -30}, {-20, 6}, {40, 6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vel_ned, discreteKalmanFilter.y_meas[4:6]) annotation(
    Line(points = {{-120, -80}, {-20, -80}, {-20, 6}, {40, 6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gps_fix, discreteKalmanFilter.correct) annotation(
    Line(points = {{50, -100}, {50, -10}}, color = {255, 0, 255}));
  annotation(
    Icon(graphics = {Text(origin = {-130, 70}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "q"), Text(origin = {-126, 6}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "acc"), Text(origin = {-126, -62}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "pos"), Text(origin = {-130, -128}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "vel"), Text(origin = {110, 18}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "pos_est"), Text(origin = {110, -64}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "vel_est"), Text(origin = {0, -76}, fillColor = {102, 102, 102}, extent = {{-100, 18}, {100, -18}}, textString = "pos/vel")}));
end PositionEstimation;
