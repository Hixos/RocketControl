within RocketControl.Rockets.Lynx.GNC.Sensors;

model TrueSensors
  extends Rockets.Internal.PartialSensorsPackage;

  parameter Boolean are_fins_present = true;
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueGyroscope trueGyroscope annotation(
    Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueAccelerometer trueAccelerometer annotation(
    Placement(visible = true, transformation(origin = {0, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueMagnetometer trueMagnetometer annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueBarometer trueBarometer annotation(
    Placement(visible = true, transformation(origin = {0, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueGNSS trueGNSS annotation(
    Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueAsset trueAsset annotation(
    Placement(visible = true, transformation(origin = {0, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression altitude_agl(y = -trueGNSS.x[3]) annotation(
    Placement(visible = true, transformation(origin = {-6, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, trueGyroscope.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, 80}, {-10, 80}}));
  connect(frame_a, trueAccelerometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, 56}, {-10, 56}}));
  connect(frame_a, trueMagnetometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(frame_a, trueBarometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, 28}, {-10, 28}}));
  connect(frame_a, trueGNSS.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, -60}, {-10, -60}}));
  connect(trueGyroscope.w, bus.w_meas) annotation(
    Line(points = {{10, 80}, {60, 80}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trueAccelerometer.a, bus.a_meas) annotation(
    Line(points = {{11, 56}, {60, 56}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trueMagnetometer.b, bus.b_meas) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trueBarometer.p, bus.p_meas) annotation(
    Line(points = {{11, 28}, {60, 28}, {60, 0}, {100, 0}}, color = {0, 0, 127}));
  connect(trueGNSS.x, bus.x_meas) annotation(
    Line(points = {{11, -56}, {60, -56}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trueGNSS.v, bus.v_meas) annotation(
    Line(points = {{11, -64}, {60, -64}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(frame_a, trueAsset.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, -18}, {-10, -18}}));
  connect(trueAsset.q, bus.q_est) annotation(
    Line(points = {{12, -18}, {60, -18}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {Text(origin = {-1, -69}, extent = {{-69, 27}, {69, -27}}, textString = "true")}));
end TrueSensors;
