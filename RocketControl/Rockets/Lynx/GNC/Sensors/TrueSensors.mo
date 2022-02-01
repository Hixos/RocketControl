within RocketControl.Rockets.Lynx.GNC.Sensors;

model TrueSensors
  extends Rockets.Internal.PartialSensorsPackage;
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.AeroStateSensor aeroStateSensor annotation(
    Placement(visible = true, transformation(origin = {-58, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueGyroscope trueGyroscope annotation(
    Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueAccelerometer trueAccelerometer annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueMagnetometer trueMagnetometer annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueBarometer trueBarometer annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueGNSS trueGNSS annotation(
    Placement(visible = true, transformation(origin = {0, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueAsset trueAsset annotation(
    Placement(visible = true, transformation(origin = {0, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, trueGyroscope.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, 80}, {-10, 80}}));
  connect(frame_a, trueAccelerometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, 40}, {-10, 40}}));
  connect(frame_a, trueMagnetometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(frame_a, trueBarometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, -40}, {-10, -40}}));
  connect(frame_a, trueGNSS.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, -80}, {-10, -80}}));
  connect(trueGyroscope.w, bus.w_meas) annotation(
    Line(points = {{10, 80}, {60, 80}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trueAccelerometer.a, bus.a_meas) annotation(
    Line(points = {{10, 40}, {60, 40}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trueMagnetometer.b, bus.b_meas) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trueBarometer.p, bus.p_meas) annotation(
    Line(points = {{10, -40}, {60, -40}, {60, 0}, {100, 0}}, color = {0, 0, 127}));
  connect(trueGNSS.x, bus.x_meas) annotation(
    Line(points = {{12, -76}, {60, -76}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trueGNSS.v, bus.v_meas) annotation(
    Line(points = {{12, -84}, {60, -84}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(frame_a, trueAsset.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, -18}, {-10, -18}}));
  connect(trueAsset.q, bus.q_est) annotation(
    Line(points = {{12, -18}, {60, -18}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(aeroStateSensor.frame_a, frame_a) annotation(
    Line(points = {{-68, 90}, {-80, 90}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end TrueSensors;
