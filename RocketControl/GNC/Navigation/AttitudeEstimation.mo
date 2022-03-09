within RocketControl.GNC.Navigation;

model AttitudeEstimation "Rocket attitude estimation from gyroscope and magnetometer data, using a Multiplicative Extended Kalamn Filter (MEKF)"
  import Modelica.Mechanics.MultiBody.Frames.Quaternions;
  extends RocketControl.Icons.Navigation;
  parameter Integer samplingPeriodMs;
  parameter SI.Angle heading0;
  parameter SI.Angle elevation0;
  parameter SI.Angle roll0 = 0;
  parameter Types.AngularVelocity bias0[3] = {0, 0, 0};
  parameter Real P0[6, 6] = identity(6);
  parameter Real sigma_v "Attitude noise";
  parameter Types.AngularVelocity sigma_u "Bias noise";
  parameter Real sigma_b "Normalized magnetomer measurement noise";
  Types.AngularVelocity bias[3](start = zeros(3));
  Modelica.Blocks.Interfaces.RealInput w_meas[3](each quantity = "AngularVelocity", each unit = "rad/s", each displayUnit = "deg/s") annotation(
    Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput b_meas[3](each final quantity = "MagneticFluxDensity", each final unit = "T", each displayUnit = "nT") annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput r_0_est[3](each quantity = "Position", each unit = "m") annotation(
    Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput q_est[4] annotation(
    Placement(visible = true, transformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput w_est[3](each final quantity = "AngularVelocity", each final unit = "rad/s", each displayUnit = "deg/s") annotation(
    Placement(visible = true, transformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
  final parameter SI.Duration T = samplingPeriodMs / 1000;
  final parameter Modelica.Mechanics.MultiBody.Frames.Orientation R0 = Modelica.Mechanics.MultiBody.Frames.axesRotations({3, 2, 1}, {heading0, elevation0, roll0}, {0, 0, 0});
  final parameter Real q0[4] = Quaternions.from_T(R0.T);
  final parameter Real x0[6] = {0, 0, 0, 0, 0, 0};
  final parameter Real R[3, 3] = identity(3) * sigma_b ^ 2;
  

  Real q_prop[4](start = q0);
  Real q_prev[4];
  Real q_update[4];
  Real q_star[4];
  SI.MagneticFluxDensity b_n[3];
  Real b_n_vers[3];
  Real b_b_vers[3];
  Real P_prop[6, 6](start = P0);
  Real P_update[6, 6];
  Real K[6, 3];
  Real H[3, 6];
  Real h[3];
  Real Aq[3, 3];
  Real x_update[6];
  Real F[6, 6];
  Real Q[6, 6];

  function Fmatrix
    input Real[3] w;
    output Real[6, 6] F;
  protected
    Real wnorm;
    Real Ftemp[6, 6];
  algorithm
    wnorm := norm(w);
    if abs(wnorm) > 1e-5 then
      F := zeros(6, 6);
      F[1:3, 1:3] := identity(3) - skew(w) * sin(wnorm * T) / wnorm + skew(w) ^ 2 * (1 - cos(wnorm * T)) / wnorm ^ 2;
      F[1:3, 4:6] := skew(w) * (1 - cos(wnorm * T)) ./ wnorm ^ 2 - identity(3) * T - skew(w) ^ 2 * (wnorm * T - sin(wnorm * T)) ./ wnorm ^ 3;
      F[4:6, 4:6] := identity(3);
    else
      Ftemp := [-skew(w), -identity(3); zeros(3, 3), zeros(3, 3)];
      F := identity(6) + T * Ftemp;
    end if;
  end Fmatrix;

  function Qmatrix
    input Real[3] w;
    input Real sigma_v;
    input Real sigma_u;
    output Real Q[6, 6];
  protected
    Real wnorm;
    Real Qtemp[6, 6];
    constant Real G[6, 6] = [-identity(3), zeros(3, 3); zeros(3, 3), identity(3)];
  algorithm
    wnorm := norm(w);
    if wnorm > 1e-5 then
      Q[1:3, 1:3] := sigma_v ^ 2 * T * identity(3) + sigma_u ^ 2 * (1 / 3 * T ^ 3 * identity(3) - skew(w) ^ 2 * (2 * wnorm * T - 2 * sin(wnorm * T) - 1 / 3 * wnorm ^ 3 * T ^ 3) / wnorm ^ 5);
      Q[1:3, 4:6] := sigma_u ^ 2 * (skew(w) * (wnorm * T - sin(wnorm * T)) / wnorm ^ 3 - 0.5 * T ^ 2 * identity(3) - skew(w) ^ 2 * (0.5 * wnorm ^ 2 * T ^ 2 + cos(wnorm * T) - 1) / wnorm ^ 4);
      Q[4:6, 1:3] := transpose(Q[1:3, 4:6]);
      Q[4:6, 4:6] := sigma_u ^ 2 * T * identity(3);
    else
      Qtemp := [identity(3) * sigma_v ^ 2, zeros(3, 3); zeros(3, 3), identity(3) * sigma_u ^ 2];
      Q := T * G * Qtemp * transpose(G);
    end if;
  end Qmatrix;

  function quatSkew
    input Real q[4];
    output Real E[4, 3];
  algorithm
    E[1, 1] := q[4];
    E[1, 2] := -q[3];
    E[1, 3] := q[2];
    E[2, 1] := q[3];
    E[2, 2] := q[4];
    E[2, 3] := -q[1];
    E[3, 1] := -q[2];
    E[3, 2] := q[1];
    E[3, 3] := q[4];
    E[4, 1] := -q[1];
    E[4, 2] := -q[2];
    E[4, 3] := -q[3];
  end quatSkew;

  function propagateQuaternion
    input Real[4] q;
    input Real[3] w;
    output Real[4] q_prop;
  protected
    Real[4, 4] O;
    Real[3] psi;
    Real wnorm;
  algorithm
    wnorm := norm(w);
    if wnorm > 1e-7 then
      psi := sin(0.5 * wnorm * T) * w / wnorm;
      O := [cos(0.5 * wnorm * T) * identity(3) - skew(psi), matrix(psi); -transpose(matrix(psi)), cos(0.5 * wnorm * T)];
    else
      O := identity(4);
    end if;
    q_prop := O * q;
  end propagateQuaternion;

  outer RocketControl.World.Interfaces.WorldBase world;
equation
  when Clock() then
    q_prev = previous(q_prop);
    q_est = q_prev;
    Aq = Quaternions.to_T(q_prev);
// Update
    b_n = world.magneticField(r_0_est);
    b_n_vers = b_n / norm(b_n);
    b_b_vers = b_meas / norm(b_meas);
    h = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(q_prev, b_n_vers);
    H = [skew(h), zeros(3, 3)];
    K = transpose(Modelica.Math.Matrices.solve2(transpose(H * previous(P_prop) * transpose(H) + R), transpose(previous(P_prop) * transpose(H))));
    P_update = (identity(6) - K * H) * previous(P_prop);
    x_update = K * (b_b_vers - h);
// Reset
    q_star = q_prev + 0.5 * quatSkew(q_prev) * x_update[1:3];
    q_update = q_star / norm(q_star);
    bias = previous(bias) + x_update[4:6];
// Propagate
    w_est = w_meas - bias;
    F = Fmatrix(w_est);
    Q = Qmatrix(w_est, sigma_v, sigma_u);
    P_prop = F * P_update * transpose(F) + Q;
    q_prop = propagateQuaternion(q_update, w_est);
  end when;
  annotation(
    Icon(graphics = {Text(origin = {134, 84}, lineColor = {111, 111, 111}, extent = {{-30, 20}, {30, -20}}, textString = "q"), Text(origin = {134, -16}, lineColor = {111, 111, 111}, extent = {{-30, 20}, {30, -20}}, textString = "w"), Text(origin = {-152, 120}, lineColor = {111, 111, 111}, extent = {{-40, 20}, {40, -20}}, textString = "w"), Text(origin = {-148, 40}, lineColor = {111, 111, 111}, extent = {{-40, 20}, {40, -20}}, textString = "b"), Text(origin = {-146, -40}, lineColor = {111, 111, 111}, extent = {{-40, 20}, {40, -20}}, textString = "r_0"), Text(origin = {0, -76}, fillColor = {102, 102, 102}, extent = {{-100, 18}, {100, -18}}, textString = "asset"), Line(origin = {-97, 0}, points = {{-3, 80}, {3, 80}, {3, -80}, {-3, -80}}), Line(origin = {-95, 0}, points = {{-5, 0}, {5, 0}}), Line(origin = {95, 25}, points = {{-5, -25}, {-1, -25}, {-1, 25}, {5, 25}}), Line(origin = {102, -25}, points = {{-8, 25}, {-8, -25}, {8, -25}})}));
end AttitudeEstimation;
