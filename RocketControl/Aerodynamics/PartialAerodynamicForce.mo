within RocketControl.Aerodynamics;

partial model PartialAerodynamicForce
  extends Internal.Icons.AerodynamicsIcon;
  import Modelica.Mechanics.MultiBody.Frames;
  type AeroCoefficient = Real(unit = "1");
  type C = Coefficients;
  outer World.Atmosphere atmosphere;
  outer World.MyWorld world;
  parameter Modelica.Units.SI.Angle max_alpha = from_deg(10);
  parameter Modelica.Units.SI.Angle min_alpha = from_deg(-10);
  parameter Modelica.Units.SI.Angle max_beta = from_deg(10);
  parameter Modelica.Units.SI.Angle min_beta = from_deg(-10);
  parameter Modelica.Units.SI.Length d = 0.15;
  parameter Modelica.Units.SI.Area S = pi * (0.15 / 2) ^ 2;
  AeroCoefficient coeffs[Coefficients];
  SI.Angle alpha0;
  SI.Angle beta0;
  AeroCoefficient CA;
  AeroCoefficient CY;
  AeroCoefficient CN;
  AeroCoefficient CLL;
  AeroCoefficient CLM;
  AeroCoefficient CLN;
  SI.Velocity v_norm;
  SI.Force fa[3];
  SI.Torque ma[3];
  Modelica.Units.SI.Pressure q(displayUnit = "Pa");
  Real q_v(unit = "kg/(m2.s)");
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Aerodynamics.Interfaces.AeroStateInput aeroState annotation(
    Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-110, -10}, {-90, 10}}, rotation = 0)));
equation
//  assert(aeroState.alpha <= max_alpha and aeroState.alpha >= min_alpha, "Angle of attack out of range");
//  assert(aeroState.beta <= max_beta and aeroState.beta >= min_beta, "Sideslip angle out of range");
// TODO: alpha0 / beta0 should be the nearest grid points in the case of nearest neighbour interpolation
  alpha0 = max(min(aeroState.alpha, max_alpha), min_alpha);
  beta0 = max(min(aeroState.beta, max_beta), min_beta);
  v_norm = norm(aeroState.v);
  q_v = 0.5 * atmosphere.density(world.altitude(frame_b.r_0)) * v_norm;
  q = q_v * v_norm;
  CA = coeffs[C.CA];
  CY = coeffs[C.CY] + coeffs[C.CYB] * (aeroState.beta - beta0);
// Second term is always zero in case of linear interpoaltion of the coefficients
  CN = coeffs[C.CN] + coeffs[C.CNA] * (aeroState.alpha - alpha0);
  CLL = coeffs[C.CLL] + coeffs[C.CLLB] * (aeroState.beta - beta0);
  CLM = coeffs[C.CM] + coeffs[C.CMA] * (aeroState.alpha - alpha0);
  CLN = coeffs[C.CLN] + coeffs[C.CLNB] * (aeroState.beta - beta0);
  fa[1] = (-q * S * CA) - q_v * S * coeffs[C.CAQ] * aeroState.w[2] * d;
  fa[2] = q * S * CY + q_v * S * (coeffs[C.CYP] * aeroState.w[1] + coeffs[C.CYR] * aeroState.w[3]) * d;
  fa[3] = (-q * S * CN) - q_v * S * (coeffs[C.CNQ] * aeroState.w[2] + coeffs[C.CNAD] * aeroState.alpha_dot) * d;
  ma[1] = q_v * S * d * (v_norm * CLL + (coeffs[C.CLLP] * aeroState.w[1] + coeffs[C.CLLR] * aeroState.w[3]) * d / 2);
  ma[2] = q_v * S * d * (v_norm * CLM + (coeffs[C.CMAD] * der(aeroState.alpha) + coeffs[C.CMQ] * aeroState.w[2]) * d / 2);
  ma[3] = q_v * S * d * (v_norm * CLN + (coeffs[C.CLNR] * aeroState.w[3] + coeffs[C.CLNP] * aeroState.w[1]) * d / 2);
  frame_b.f = -fa;
  frame_b.t = -ma;
  annotation(
    Icon(graphics = {Text(origin = {2, -178}, lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name")}));
end PartialAerodynamicForce;
