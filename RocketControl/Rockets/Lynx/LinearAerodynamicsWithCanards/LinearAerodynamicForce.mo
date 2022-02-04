within RocketControl.Rockets.Lynx.LinearAerodynamicsWithCanards;

model LinearAerodynamicForce
  extends RocketControl.Aerodynamics.PartialLinearAerodynamicForce;
  parameter Integer nCAO(min = 1) annotation(Evaluate = true);
  
  parameter Real machCA0[nCAO];
  parameter Real CA0[nCAO];
  
  parameter Real CA_a;
  parameter Real CA_b;
  parameter Real CA_dy;
  parameter Real CA_dp;
  parameter Real CA_dr;
  parameter Real CA_ds;
  parameter Real CN_a;
  parameter Real CN_dp;
  parameter Real CY_b;
  parameter Real CY_dy;
  parameter Real CLL_dr;
  parameter Real CLM_a;
  parameter Real CLM_dp;
  parameter Real CLN_b;
  parameter Real CLN_dy;

  Real CA0_int;
  
  Modelica.Blocks.Interfaces.RealInput finDeflection[4](each final unit = "rad", each final quantity="Angle", each displayUnit="deg") annotation(
    Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.Deflection2Control cu annotation(
    Placement(visible = true, transformation(origin = {-50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  if nCAO > 1 then 
  CA0_int = Modelica.Math.Vectors.interpolate(machCA0, CA0, aeroState.mach);
  else
    CA0_int = CA0[1];
  end if;
  CA = CA0_int +  CA_dy*cu.control[1]^2 + CA_dp*cu.control[2]^2 + CA_dr*cu.control[3]^2 + CA_ds*cu.control[4]^2;
  
  CN = CN_a*aeroState.alpha + CN_dp*cu.control[2];
  CY = CY_b*aeroState.beta + CY_dy*cu.control[1];
  
  CLL = CLL_dr*cu.control[3];
  CLM = CLM_a*aeroState.alpha + CLM_dp*cu.control[2];
  CLN = CLN_b*aeroState.beta + CLN_dy*cu.control[1];
  
  connect(finDeflection, cu.u) annotation(
    Line(points = {{-100, -70}, {-62, -70}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end LinearAerodynamicForce;
