within RocketControl.GNC.Control.LinearStateMatrices;

block RocketAndActuator
  extends Internal.PartialLinearStateMatrix(nx=8,nu=3);
  outer RocketControl.World.Atmosphere atmosphere;
  parameter Real CA0;
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
  parameter Real CY_r;
  parameter Real CN_q;
  parameter Real CLL_p;
  parameter Real CLM_q;
  parameter Real CLN_r;
  
  parameter Real S;
  parameter Real c;
  parameter Real m;
  parameter Real Ix;
  parameter Real Is;
  parameter Real wa;
  SI.Velocity u;
  SI.Velocity v;
  SI.Velocity w;
  SI.AngularVelocity p;
  SI.AngularVelocity q;
  SI.AngularVelocity r;
  SI.Angle dys;
  SI.Angle dps;
  SI.Angle drs;
  SI.Density rho(displayUnit = "kg/m3");
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-110, -10}, {-90, 10}}, rotation = 0)));
    
  SI.Velocity v_body[3];
equation
  v_body = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, bus.v_est);
  rho = atmosphere.density(-bus.x_est[3]);
  u = v_body[1];
  v = 0;//v_body[2];
  w = 0;//v_body[3];
  p = 0;//bus.w_est[1];
  q = 0;//bus.w_est[2];
  r = 0;//bus.w_est[3];
  dys = 0;//bus.control_position_meas[1];
  dps = 0;//bus.control_position_meas[2];
  drs = 0;//bus.control_position_meas[3];
  A = [    (CY_b*S*rho*u)/(2*m),                        0,                          0,                          0, (CY_r*S*c*rho*u)/(2*m) - u,     (CY_dy*S*rho*u^2)/(2*m),                           0,                           0;
                        0,    -(CN_a*S*rho*u)/(2*m),                          0, u - (CN_q*S*c*rho*u)/(2*m),                          0,                           0,    -(CN_dp*S*rho*u^2)/(2*m),                           0;
                        0,                        0, (CLL_p*S*c^2*rho*u)/(4*Ix),                          0,                          0,                           0,                           0, (CLL_dr*S*c*rho*u^2)/(2*Ix);
                        0, (CLM_a*S*c*rho*u)/(2*Is),                          0, (CLM_q*S*c^2*rho*u)/(4*Is),                          0,                           0, (CLM_dp*S*c*rho*u^2)/(2*Is),                           0;
 (CLN_b*S*c*rho*u)/(2*Is),                        0,                          0,                          0, (CLN_r*S*c^2*rho*u)/(4*Is), (CLN_dy*S*c*rho*u^2)/(2*Is),                           0,                           0;
                        0,                        0,                          0,                          0,                          0,                         -wa,                           0,                           0;
                        0,                        0,                          0,                          0,                          0,                           0,                         -wa,                           0;
                        0,                        0,                          0,                          0,                          0,                           0,                           0,                         -wa];
 
  B = [0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; wa, 0, 0; 0, wa, 0; 0, 0, wa];

  annotation(
    Icon,
    Diagram);
end RocketAndActuator;
