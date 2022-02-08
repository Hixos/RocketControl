within RocketControl.GNC.Control.LinearStateMatrices;

block RocketAndActuator
  extends Internal.PartialLinearStateMatrix(nx=9,nu=3);
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
  q = bus.w_est[2];
  r = bus.w_est[3];
  dys = 0;//bus.control_position_meas[1];
  dps = 0;//bus.control_position_meas[2];
  drs = 0;//bus.control_position_meas[3];
  A = [-CA0 * S * rho * u / m, r - CA_b * S * rho * v / m, (-q) - CA_a * S * rho * w / m, 0, -w, v, 0, 0, 0; CY_b * S * rho * v / (2 * m) - r + CY_dy * S * dys * rho * u / m, CY_b * S * rho * u / (2 * m), p, w, 0, -u, CY_dy * S * rho * u ^ 2 / (2 * m), 0, 0; q - CN_a * S * rho * w / (2 * m) - CN_dp * S * dps * rho * u / m, -p, -CN_a * S * rho * u / (2 * m), -v, u, 0, 0, -CN_dp * S * rho * u ^ 2 / (2 * m), 0; CLL_dr * S * c * drs * rho * u / Ix, 0, 0, 0, 0, 0, 0, 0, CLL_dr * S * c * rho * u ^ 2 / (2 * Ix); CLM_a * S * c * rho * w / (2 * Is) + CLM_dp * S * c * dps * rho * u / Is, 0, CLM_a * S * c * rho * u / (2 * Is), r - Ix * r / Is, 0, p - Ix * p / Is, 0, CLM_dp * S * c * rho * u ^ 2 / (2 * Is), 0; CLN_b * S * c * rho * v / (2 * Is) + CLN_dy * S * c * dys * rho * u / Is, CLN_b * S * c * rho * u / (2 * Is), 0, Ix * q / Is - q, Ix * p / Is - p, 0, CLN_dy * S * c * rho * u ^ 2 / (2 * Is), 0, 0; 0, 0, 0, 0, 0, 0, -wa, 0, 0; 0, 0, 0, 0, 0, 0, 0, -wa, 0; 0, 0, 0, 0, 0, 0, 0, 0, -wa];
  B = [0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; wa, 0, 0; 0, wa, 0; 0, 0, wa];
  annotation(
    Icon,
    Diagram);
end RocketAndActuator;
