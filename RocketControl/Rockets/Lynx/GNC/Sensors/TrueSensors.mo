within RocketControl.Rockets.Lynx.GNC.Sensors;

model TrueSensors
  extends Rockets.Internal.PartialSensorsPackage;
  parameter Boolean are_fins_present = true;
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.AeroStateSensor aeroStateSensor annotation(
    Placement(visible = true, transformation(origin = {-58, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
  Modelica.Blocks.Interfaces.RealInput fin_disable[4] if not are_fins_present annotation(
    Placement(visible = true, transformation(origin = {-54, -90}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-42, -86}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant fin_zero(k = {0, 0, 0, 0}, n = 4)  annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueFinPositionSensor trueFinPositionSensor annotation(
    Placement(visible = true, transformation(origin = {0, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
  connect(aeroStateSensor.frame_a, frame_a) annotation(
    Line(points = {{-68, 90}, {-80, 90}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(fin_zero.v, fin_disable) annotation(
    Line(points = {{-79, -90}, {-54, -90}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fin_disable, trueFinPositionSensor.fin_pos_true) annotation(
    Line(points = {{-54, -90}, {-33, -90}, {-33, -86}, {-12, -86}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trueFinPositionSensor.control_meas, bus.control_position_meas) annotation(
    Line(points = {{12, -86}, {60, -86}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.fin_true_position, trueFinPositionSensor.fin_pos_true) annotation(
    Line(points = {{100, 0}, {96, 0}, {96, -100}, {-12, -100}, {-12, -86}}, thickness = 0.5));
  annotation(
    Icon(graphics = {Text(origin = {-1, -69}, extent = {{-69, 27}, {69, -27}}, textString = "true")}));
end TrueSensors;
