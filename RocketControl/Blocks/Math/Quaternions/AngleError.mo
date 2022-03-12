within RocketControl.Blocks.Math.Quaternions;

model AngleError
  extends RocketControl.Icons.QuaternionBlock;
  import RocketControl.Math.Quaternions.*;
  Modelica.Blocks.Interfaces.RealInput q1[4] annotation(
    Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 42}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q2[4] annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput angle_err(unit="rad", quantity="Angle", displayUnit="deg") annotation(
    Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Real qerr[4];
  Modelica.Blocks.Interfaces.RealOutput e[3]()     annotation(
    Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    Real qerr_prev[4];
    Real s(start = 1);
equation
qerr_prev = previous(qerr);
  qerr = quatmolt(q1, quatinv(q2));
  
  if noEvent(sign(qerr_prev[4]) <> sign(qerr[4]) and abs(qerr[4] - qerr_prev[4]) > 1) then
    s = -previous(s);
    else
    
    s = previous(s);
    end if;
    
    
  angle_err = 2*acos(s*qerr[4]);
  if angle_err > 1e-4 then
  e = s*qerr[1:3]/sin(angle_err/2);
  else
    e = {0,0,1};
  end if;
annotation(
    Icon(graphics = {Text(origin = {-2, 8}, extent = {{-88, 64}, {88, -64}}, textString = "θerr")}));
end AngleError;
