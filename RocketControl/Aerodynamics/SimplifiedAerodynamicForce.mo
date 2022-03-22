within RocketControl.Aerodynamics;

model SimplifiedAerodynamicForce
extends Icons.AerodynamicsIcon; 

  outer World.Atmosphere atmosphere;
  outer World.Interfaces.WorldBase world;
  
  parameter SI.Area S = pi * (0.15 / 2) ^ 2;
  parameter SI.Length d = 0.15;
  parameter Modelica.Units.SI.Angle max_alpha = from_deg(20);
  parameter Modelica.Units.SI.Angle max_beta = from_deg(20);
  parameter Real angular_damping_reverse = 2;
  parameter Integer nCAO(min = 1) annotation(Evaluate = true);
  
  parameter Real machCA0[nCAO];
  parameter Real CA0[nCAO];
  
  parameter Real CA_a;
  parameter Real CA_b;
  parameter Real CA_dp;
  parameter Real CA_dr;
  parameter Real CA_ds;
  parameter Real CA_dy;
  parameter Real CLL_dr;
  parameter Real CLL_p ;
  parameter Real CLM_a;
  parameter Real CLM_q;
  parameter Real CLM_dp;
  parameter Real CLN_b;
  parameter Real CLN_r;
  parameter Real CLN_dy;
  parameter Real CN_a;
  parameter Real CN_q;
  parameter Real CN_dp;
  parameter Real CY_b;
  parameter Real CY_r;
  parameter Real CY_dy;
  
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Interfaces.AeroStateInput aeroState annotation(
    Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-110, -10}, {-90, 10}}, rotation = 0)));
    
    SI.Velocity v_norm;
    Real q_v;
    SI.Pressure q(displayUnit = "Pa");
  
    SI.Torque ma[3];
    SI.Force fa[3];
    
    Real CA0_int;
    Real CA;
    Real CY;
    Real CN;
    Real CLL;
    Real CLM;
    Real CLN;
  GNC.Control.Deflection2Control cu annotation(
    Placement(visible = true, transformation(origin = {-50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput finDeflection[4](each displayUnit = "deg", each final quantity = "Angle", each final unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanInput chute_open annotation(
    Placement(visible = true, transformation(origin = {-100, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-98, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  v_norm = norm(aeroState.v);
  q_v = 0.5 * atmosphere.density(world.altitude(frame_b.r_0)) * v_norm;
  q = q_v * v_norm;
  if nCAO > 1 then
    CA0_int = Modelica.Math.Vectors.interpolate(machCA0, CA0, min(aeroState.mach, machCA0[end]));
  else
    CA0_int = CA0[1];
  end if;
  CA = CA0_int + CA_dy * cu.control[1] ^ 2 + CA_dp * cu.control[2] ^ 2 + CA_dr * cu.control[3] ^ 2 + CA_ds * cu.control[4] ^ 2;
  CN = CN_a * aeroState.alpha + CN_dp * cu.control[2];
  CY = CY_b * aeroState.beta + CY_dy * cu.control[1];
  CLL = CLL_dr * cu.control[3];
  CLM = CLM_a * aeroState.alpha + CLM_dp * cu.control[2];
  CLN = CLN_b * aeroState.beta + CLN_dy * cu.control[1];
  
  fa[1] = -q * S * CA;
  fa[2] = q * S * CY + q_v*S*CY_r*aeroState.w[3]*d;
  fa[3] = -q * S * CN - q_v*S*CN_q*aeroState.w[2]*d;
  
  ma[1] = q * S * d * CLL + q_v*S*d*CLL_p*aeroState.w[1]*d/2;
  ma[2] = q * S * d * CLM + q_v*S*d*CLM_q*aeroState.w[2]*d/2;
  ma[3] = q * S * d * CLN + q_v*S*d*CLN_r*aeroState.w[3]*d/2;
  
//  if abs(aeroState.alpha) < max_alpha and abs(aeroState.beta) < max_beta then
if not chute_open then
  frame_b.f = -fa;
  frame_b.t = -ma;
  else
  frame_b.f = zeros(3);
  frame_b.t[1] = -q_v*S*d*CLL_p*aeroState.w[1]*d/2;
  frame_b.t[2] = -q_v*S*d*CLM_q*aeroState.w[2]*d/2;
  frame_b.t[3] = -q_v*S*d*CLN_r*aeroState.w[3]*d/2;
  end if;
  connect(finDeflection, cu.u) annotation(
    Line(points = {{-100, -70}, {-62, -70}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {Text(origin = {1, -49}, extent = {{-89, 23}, {89, -23}}, textString = "linear")}));
end SimplifiedAerodynamicForce;
