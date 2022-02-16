within RocketControl.Rockets.Lynx;

model LynxLinearAeroDiscrete
  extends Rockets.Internal.PartialRocket;
  RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Blocks.Math.Vectors.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4)  annotation(
    Placement(visible = true, transformation(origin = {-12, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Clocked.VectorHold vectorHold(n = 4)  annotation(
    Placement(visible = true, transformation(origin = {72, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Components.Actuators.TFServoMotor tFServoMotor(a = {0.0769, 1}, saturation_angle(displayUnit = "deg") = 0.1745329251994329)  annotation(
    Placement(visible = true, transformation(origin = {26, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Components.Clocked.VectorSample vectorSample(n = 4)  annotation(
    Placement(visible = true, transformation(origin = {36, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.Deflection2Control deflection2Control annotation(
    Placement(visible = true, transformation(origin = {68, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LinearAerodynamicsWithCanards.LinearAerodynamics simpleAerodynamics annotation(
    Placement(visible = true, transformation(origin = {-20, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
  connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
    Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
  connect(lynxBody.ref_center, ref_center) annotation(
    Line(points = {{-40, 0}, {100, 0}}));
  connect(bus.fin_setpoint, vectorHold.u) annotation(
    Line(points = {{100, 90}, {100, 20}, {84, 20}}, thickness = 0.5));
  connect(tFServoMotor.servo_pos, bus.fin_true_position) annotation(
    Line(points = {{16, 20}, {0, 20}, {0, 56}, {100, 56}, {100, 90}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorSample.u, tFServoMotor.servo_pos) annotation(
    Line(points = {{24, 76}, {0, 76}, {0, 20}, {16, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorSample.y, deflection2Control.u) annotation(
    Line(points = {{48, 76}, {56, 76}}, color = {0, 0, 127}, thickness = 0.5));
  connect(deflection2Control.control, bus.control_position_meas) annotation(
    Line(points = {{80, 76}, {100, 76}, {100, 90}}, color = {0, 0, 127}, thickness = 0.5));
  connect(tFServoMotor.setpoint, vectorHold.y) annotation(
    Line(points = {{38, 20}, {62, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(simpleAerodynamics.frame_b, lynxBody.ref_center) annotation(
    Line(points = {{-30, 72}, {-40, 72}, {-40, 0}}, color = {95, 95, 95}));
  connect(simpleAerodynamics.finDeflection, tFServoMotor.servo_pos_nonsat) annotation(
    Line(points = {{-30, 66}, {-40, 66}, {-40, 14}, {16, 14}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end LynxLinearAeroDiscrete;
