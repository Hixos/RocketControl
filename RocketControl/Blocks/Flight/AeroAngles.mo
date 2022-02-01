within RocketControl.Blocks.Flight;

block AeroAngles
  parameter SI.Velocity v_small = 1e-3;
  extends Icon;
  Modelica.Blocks.Interfaces.RealInput v_body[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput alpha(displayUnit = "deg", quantity = "Angle", unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput beta(displayUnit = "deg", quantity = "Angle", unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
   if noEvent(abs(v_body[1]) > v_small) then
    alpha = atan(v_body[3] / v_body[1]);
  elseif noEvent(abs(v_body[3]) > abs(v_body[1])) then
    alpha = sign(v_body[3]) * pi / 2;
  else
    alpha = 0;
  end if;
  
  if noEvent(abs(v_body[1]) > v_small) then
  beta = atan(v_body[2] / v_body[1]);
//sideslip = asin(v_b[2] / v_norm);
  elseif noEvent(abs(v_body[2]) > v_small) then
  beta = Modelica.Units.Conversions.from_deg(90 * sign(v_body[2]));
  else
  beta = 0;
  end if;
  annotation(
    Icon(graphics = {Line(origin = {-2.81, 9.13}, points = {{26.8055, 74.8696}, {-79.1945, -49.1304}, {78.8055, -75.1304}}, pattern = LinePattern.Dash, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 12), Line(origin = {-10, -7}, points = {{-72, -33}, {72, 33}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Line(origin = {-9.2, 16.12}, points = {{5.20429, -20.1221}, {7.20429, -8.12209}, {3.20429, 5.87791}, {-6.79571, 19.8779}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Open}, arrowSize = 12), Line(origin = {13.94, -31.07}, points = {{-17.9418, 27.0743}, {-1.94184, 25.0743}, {12.0582, 15.0743}, {18.0582, -0.925701}, {18.0582, -14.9257}, {14.0582, -26.9257}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Open}, arrowSize = 12), Text(origin = {-116, 39}, lineColor = {102, 102, 102}, extent = {{-64, 21}, {64, -21}}, textString = "v_body"), Text(origin = {100, 83}, lineColor = {102, 102, 102}, extent = {{-64, 21}, {64, -21}}, textString = "alpha"), Text(origin = {104, -77}, lineColor = {102, 102, 102}, extent = {{-64, 21}, {64, -21}}, textString = "beta")}));
end AeroAngles;
