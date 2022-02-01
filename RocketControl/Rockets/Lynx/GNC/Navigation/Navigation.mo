within RocketControl.Rockets.Lynx.GNC.Navigation;

model Navigation
  outer World.SimOptions opt;
  extends RocketControl.Rockets.Internal.PartialNavigationSystem;
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.AttitudeEstimation attitudeEstimation(elevation0 = 1.466076571675237, heading0 = 2.268928027592628, samplingPeriodMs = opt.samplePeriodMs, sigma_b = 2, sigma_u = from_deg(10), sigma_v = from_deg(60)) annotation(
    Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.PositionEstimation positionEstimation(samplingPeriodMs = opt.samplePeriodMs, sigma_gps = {1000, 1000, 1000, 100, 100, 100}, sigma_pos = 2, sigma_vel = 1) annotation(
    Placement(visible = true, transformation(origin = {-48, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Blocks.Flight.Downrange downrange annotation(
    Placement(visible = true, transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Blocks.Flight.Track track annotation(
    Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Quaternions.Quaternion2Euler quaternion2Euler annotation(
    Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Blocks.Flight.ClimbAngle climbAngle annotation(
    Placement(visible = true, transformation(origin = {30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Trasformations.ned2body v_body annotation(
    Placement(visible = true, transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Blocks.Flight.AeroAngles aeroAngles annotation(
    Placement(visible = true, transformation(origin = {-30, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus.w_meas, attitudeEstimation.w_meas) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 58}, {-62, 58}}, thickness = 0.5));
  connect(bus.b_meas, attitudeEstimation.b_meas) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 50}, {-62, 50}}, thickness = 0.5));
  connect(attitudeEstimation.q_est, bus.q_est) annotation(
    Line(points = {{-39, 55}, {60, 55}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(attitudeEstimation.w_est, bus.w_est) annotation(
    Line(points = {{-39, 45}, {60, 45}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.q_est, positionEstimation.q) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 20}, {-60, 20}}, thickness = 0.5));
  connect(bus.a_meas, positionEstimation.acc_body) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 14}, {-60, 14}}, thickness = 0.5));
  connect(bus.x_meas, positionEstimation.pos_ned) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 6}, {-60, 6}}, thickness = 0.5));
  connect(bus.v_meas, positionEstimation.vel_ned) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 0}, {-60, 0}}, thickness = 0.5));
  connect(bus.x_est, attitudeEstimation.r_0_est) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 42}, {-62, 42}}, thickness = 0.5));
  connect(positionEstimation.pos_est, bus.x_est) annotation(
    Line(points = {{-36, 14}, {-14, 14}, {-14, 34}, {60, 34}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(positionEstimation.vel_est, bus.v_est) annotation(
    Line(points = {{-36, 6}, {-8, 6}, {-8, 26}, {60, 26}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.q_est, quaternion2Euler.q) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -16}, {0, -16}, {0, 0}, {18, 0}}, thickness = 0.5));
  connect(bus.v_est, track.v) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -30}, {18, -30}}, thickness = 0.5));
  connect(track.v, climbAngle.v) annotation(
    Line(points = {{18, -30}, {0, -30}, {0, -60}, {18, -60}}, color = {0, 0, 127}, thickness = 0.5));
  connect(climbAngle.v, downrange.x) annotation(
    Line(points = {{18, -60}, {0, -60}, {0, -90}, {18, -90}}, color = {0, 0, 127}, thickness = 0.5));
 connect(bus.v_est, v_body.x_w) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -44}, {-82, -44}}, thickness = 0.5));
 connect(bus.q_est, v_body.q_bw) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -56}, {-82, -56}}, thickness = 0.5));
 connect(v_body.x_b, aeroAngles.v_body) annotation(
    Line(points = {{-58, -50}, {-42, -50}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Text(origin = {-10, -254}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
end Navigation;
