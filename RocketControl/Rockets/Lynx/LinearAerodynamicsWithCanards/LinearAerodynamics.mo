within RocketControl.Rockets.Lynx.LinearAerodynamicsWithCanards;

model LinearAerodynamics
  parameter Integer nCAO(min = 1) = 6 annotation(Evaluate = true);
  
  parameter Real machCA0[nCAO] = {0, 0.2, 0.5, 0.7, 0.85, 1};
  parameter Real CA0[nCAO] = {0.4200, 0.4200, 0.3780, 0.3620, 0.3830, 0.6050};
  
  parameter Real CA_a = -0.1725;
  parameter Real CA_b = -0.1725;
  parameter Real CA_dp = 0.4001;
  parameter Real CA_dr = 0.5739;
  parameter Real CA_ds = 0.8899;
  parameter Real CA_dy = 0.4001;
  parameter Real CLL_dr = 2.3963;
  parameter Real CLL_p = -19;
  parameter Real CLM_a = -37.2959;
  parameter Real CLM_q = -1813;
  parameter Real CLM_dp = 21.8445;
  parameter Real CLN_b = 37.2959;
  parameter Real CLN_r = -1813;
  parameter Real CLN_dy = 21.8445;
  parameter Real CN_a = 24.0744;
  parameter Real CN_q = 154;
  parameter Real CN_dp = 3.4045;
  parameter Real CY_b = -24.0744;
  parameter Real CY_r = 154;
  parameter Real CY_dy = 3.4045;

  
  extends RocketControl.Aerodynamics.PartialAerodynamics(redeclare RocketControl.Aerodynamics.SimplifiedAerodynamicForce aerodynamicForce(nCAO = nCAO, machCA0 = machCA0, CA0 = CA0, CA_a = CA_a, CA_b = CA_b, CA_dp = CA_dp, CA_dr = CA_dr, CA_ds = CA_ds, CA_dy = CA_dy, CLL_dr = CLL_dr, CLL_p  = CLL_p , CLM_a = CLM_a, CLM_q = CLM_q, CLM_dp = CLM_dp, CLN_b = CLN_b, CLN_r = CLN_r, CLN_dy = CLN_dy, CN_a = CN_a, CN_q = CN_q, CN_dp = CN_dp, CY_b = CY_b, CY_r = CY_r, CY_dy = CY_dy));
  
  Modelica.Blocks.Interfaces.RealInput finDeflection[4] annotation(
    Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(finDeflection, aerodynamicForce.finDeflection) annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end LinearAerodynamics;
