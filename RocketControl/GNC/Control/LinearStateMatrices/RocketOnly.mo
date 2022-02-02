within RocketControl.GNC.Control.LinearStateMatrices;

model RocketOnly
  extends Internal.PartialLinearStateMatrix(nx=6,nu=3);
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
  
  SI.Velocity v_body[3];
  Real u;
  Real v;
  Real w;
  Real p;
  Real q;
  Real r;
  Real rho;
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-110, -10}, {-90, 10}}, rotation = 0)));
equation
  v_body = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, bus.v_est);
  rho = atmosphere.density(-bus.x_est[3]);
  u = v_body[1];
  v = 0;//v_body[2];
  w = 0;//v_body[3];
  p = 0;//bus.w_est[1];
  q = 0;//bus.w_est[2];
  r = 0;//bus.w_est[3];
  A = [-CA0 * S * rho * u / m, r - CA_b * S * rho * v / m, (-q) - CA_a * S * rho * w / m, 0, -w, v; CY_b * S * rho * v / (2 * m) - r, CY_b * S * rho * u / (2 * m), p, w, 0, -u; q - CN_a * S * rho * w / (2 * m), -p, -CN_a * S * rho * u / (2 * m), -v, u, 0; 0, 0, 0, 0, 0, 0; CLM_a * S * c * rho * w / (2 * Is), 0, CLM_a * S * c * rho * u / (2 * Is), r - Ix * r / Is, 0, p - Ix * p / Is; CLN_b * S * c * rho * v / (2 * Is), CLN_b * S * c * rho * u / (2 * Is), 0, Ix * q / Is - q, Ix * p / Is - p, 0];
  B = [0, 0, 0; CY_dy * S * rho * u ^ 2 / (2 * m), 0, 0; 0, -CN_dp * S * rho * u ^ 2 / (2 * m), 0; 0, 0, CLL_dr * S * c * rho * u ^ 2 / (2 * Ix); 0, CLM_dp * S * c * rho * u ^ 2 / (2 * Is), 0; CLN_dy * S * c * rho * u ^ 2 / (2 * Is), 0, 0];
  annotation(
    Icon,
    Diagram);
end RocketOnly;
