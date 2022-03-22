within RocketControl.World.Atmosphere2;

model WindTurbolence
  outer RocketControl.World.Interfaces.WorldBase world;
  
  import Modelica.Constants.pi;
  parameter SI.Velocity W20;
  parameter SI.Length b;
  parameter SI.Duration noise_sample_period = 0.05;
  parameter Integer q_sign = 1;
  parameter Integer r_sign = 1;
  RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hu(n = 1) annotation(
    Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Noise.BandLimitedWhiteNoise wn_u(enableNoise = true, fixedLocalSeed = 12345, noisePower = pi, samplePeriod = 0.05, useAutomaticLocalSeed = false, useGlobalSeed = false) annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SI.Position h;
  SI.Length L_low[3];
  SI.Velocity sigma_low[3];
  //    SI.Length L_high[3];
  //    SI.Velocity sigma_high[3];
  SI.Length L_int[3];
  SI.Velocity sigma_int[3];
  RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hv(n = 2) annotation(
    Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hw(n = 2) annotation(
    Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SI.Velocity V;
  Modelica.Blocks.Noise.BandLimitedWhiteNoise wn_v(enableNoise = true, noisePower = pi, samplePeriod = noise_sample_period, useAutomaticLocalSeed = true, useGlobalSeed = true) annotation(
    Placement(visible = true, transformation(origin = {-50, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Noise.BandLimitedWhiteNoise wn_w(enableNoise = true, noisePower = pi, samplePeriod = noise_sample_period, useAutomaticLocalSeed = true, useGlobalSeed = true) annotation(
    Placement(visible = true, transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  RocketControl.Blocks.Trasformations.Units.ft2m ft2m(n = 3) annotation(
    Placement(visible = true, transformation(origin = {70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hp(n = 1) annotation(
    Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hr(n = 1) annotation(
    Placement(visible = true, transformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hq(n = 1) annotation(
    Placement(visible = true, transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Noise.BandLimitedWhiteNoise wn_p(enableNoise = true, fixedLocalSeed = 1233142, noisePower = pi, samplePeriod = noise_sample_period, useAutomaticLocalSeed = false, useGlobalSeed = false) annotation(
    Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput v_turb[3] annotation(
    Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput w_turb[3] annotation(
    Placement(visible = true, transformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v_wind[3] annotation(
    Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
protected
  final parameter Real b_ft = b / 0.3048;
equation
  V = if norm(der(frame_a.r_0) - v_wind) / 0.3048 > 1 then norm(der(frame_a.r_0) - v_wind) / 0.3048 else 1;
  
  h = world.altitude_agl(frame_a.r_0) / 0.3048;
  L_low[3] = h / 2;
  L_low[1] = h / (0.177 + 0.000823 * h) ^ 1.2;
  L_low[2] = L_low[1] / 2;
  sigma_low[3] = 0.1 * W20 / 0.3048;
  sigma_low[1] = sigma_low[3] / (0.177 + 0.000823 * h) ^ 0.4;
  sigma_low[2] = sigma_low[1];
//  L_high[1] = 1750;
//  L_high[2] = 1750 / 2;
//  L_high[3] = 1750 / 2;
  L_int = L_low;
  sigma_int = sigma_low;
// Velocity
  Hu.b = sigma_int[1] * sqrt(2 * L_int[1] / (pi * V)) * {0, 1};
  Hu.a = {L_int[1] / V, 1};
  Hv.b = sigma_int[2] * sqrt(2 * L_int[2] / (pi * V)) * {0, 2 * sqrt(3) * L_int[2] / V, 1};
  Hv.a = {(2 * L_int[2] / V) ^ 2, 2 * (2 * L_int[2] / V), 1};
  Hw.b = sigma_int[3] * sqrt(2 * L_int[3] / (pi * V)) * {0, 2 * sqrt(3) * L_int[3] / V, 1};
  Hw.a = {(2 * L_int[3] / V) ^ 2, 2 * (2 * L_int[3] / V), 1};
// Angular Velocity
  Hp.b = sigma_int[3] * sqrt(0.8 / V) * {0, (pi / (4 * b_ft)) ^ (1 / 6)};
  Hp.a = (2 * L_int[3]) ^ (1 / 3) * {4 * b_ft / pi, 1};
  Hq.b = {q_sign * 1 / V, 0};
  Hq.a = {4 * b_ft / (pi * V), 1};
  Hr.b = {r_sign * 1 / V, 0};
  Hr.a = {3 * b_ft / (pi * V), 1};
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  connect(wn_w.y, Hw.u) annotation(
    Line(points = {{-39, 30}, {-12, 30}}, color = {0, 0, 127}));
  connect(wn_v.y, Hv.u) annotation(
    Line(points = {{-39, 60}, {-12, 60}}, color = {0, 0, 127}));
  connect(wn_u.y, Hu.u) annotation(
    Line(points = {{-39, 90}, {-12, 90}}, color = {0, 0, 127}));
  connect(Hu.y, ft2m.ft[1]) annotation(
    Line(points = {{11, 90}, {40, 90}, {40, 60}, {58, 60}}, color = {0, 0, 127}));
  connect(Hv.y, ft2m.ft[2]) annotation(
    Line(points = {{11, 60}, {58, 60}}, color = {0, 0, 127}));
  connect(Hw.y, ft2m.ft[3]) annotation(
    Line(points = {{11, 30}, {40, 30}, {40, 60}, {58, 60}}, color = {0, 0, 127}));
  connect(Hw.y, Hq.u) annotation(
    Line(points = {{12, 30}, {20, 30}, {20, -30}, {-20, -30}, {-20, -50}, {-12, -50}}, color = {0, 0, 127}));
  connect(Hv.y, Hr.u) annotation(
    Line(points = {{12, 60}, {22, 60}, {22, -70}, {-20, -70}, {-20, -90}, {-12, -90}}, color = {0, 0, 127}));
  connect(ft2m.m, v_turb) annotation(
    Line(points = {{82, 60}, {110, 60}}, color = {0, 0, 127}, thickness = 0.5));
  connect(Hp.y, w_turb[1]) annotation(
    Line(points = {{12, -10}, {60, -10}, {60, -50}, {110, -50}}, color = {0, 0, 127}));
  connect(Hq.y, w_turb[2]) annotation(
    Line(points = {{12, -50}, {110, -50}}, color = {0, 0, 127}));
  connect(Hr.y, w_turb[3]) annotation(
    Line(points = {{12, -90}, {60, -90}, {60, -50}, {110, -50}}, color = {0, 0, 127}));
  connect(wn_p.y, Hp.u) annotation(
    Line(points = {{-38, -10}, {-12, -10}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end WindTurbolence;
