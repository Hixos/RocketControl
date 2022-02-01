within RocketControl.Components.Sensors.TrueSensors;

block TrueFinPositionSensor
 extends Modelica.Icons.RoundSensor;
  Modelica.Blocks.Interfaces.RealOutput control_meas[4](each displayUnit = "deg/s", each final quantity = "AngularVelocity", each final unit = "deg/s") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput fin_pos_true[4](each displayUnit = "deg/s", each final quantity = "AngularVelocity", each final unit = "deg/s") annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.GNC.Control.Deflection2Control deflection2Control annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fin_pos_true, deflection2Control.u) annotation(
    Line(points = {{-120, 0}, {-12, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(deflection2Control.control, control_meas) annotation(
    Line(points = {{12, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {Line(origin = {-85, 0}, points = {{-15, 0}, {15, 0}}), Line(origin = {85, 0}, points = {{-15, 0}, {15, 0}}), Text(origin = {-2, -200},lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name")}));
end TrueFinPositionSensor;
