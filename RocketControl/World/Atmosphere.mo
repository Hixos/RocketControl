within RocketControl.World;

model Atmosphere
  import Modelica.Constants.R;
  import RocketControl.Types.LapseRate;
  outer RocketControl.World.FlatWorld world;
  parameter SI.Temperature T0 = 288.15;
  parameter SI.Pressure P0 = 101325;
  parameter LapseRate lapse_rate = -0.0065;
  parameter SI.Density rho0 = 1.2250;
  parameter SI.MolarMass M = 0.0289644;
  parameter SI.Acceleration g0 = 9.80665;
  parameter Boolean wind_turbolence_enable = true;
  parameter SI.Length wing_span;
  parameter SI.Duration wind_noise_sample_period = 0.1;
  parameter Integer wind_q_sign = 1;
  parameter Integer wind_r_sign = 1;
  parameter Integer wind_severity_high(min = 0, max = 6) = 1;
  parameter Integer num_wind_layers(min = 1) = 1;
  parameter SI.Length wind_layer_height[num_wind_layers] = {100000};
  parameter SI.Angle wind_direction[num_wind_layers](each displayUnit = "deg") = {0};
  parameter SI.Velocity wind_magnitude[num_wind_layers] = {0};

  function density
    input SI.Position h;
    output SI.Density density;
  algorithm
    density := rho0 * (T0 / (T0 + h * lapse_rate)) ^ (1 + g0 * M / (R * lapse_rate));
  end density;

  function temperature
    input SI.Position h;
    output SI.Temperature temp;
  algorithm
    temp := T0 + h * lapse_rate;
  end temperature;

  function pressure
    input SI.Position h;
    output SI.Pressure press;
  algorithm
    press := P0 * ((T0 + h * lapse_rate) / T0) ^ (-g0 * M / (R * lapse_rate));
    annotation(
      Inline = true);
  end pressure;

  function speedOfSound
    input SI.Position h;
    output SI.Velocity speedOfSound;
  protected
    SI.Temperature T;
  algorithm
    T := temperature(h);
    speedOfSound := sqrt(1.4 * R * T / M);
  end speedOfSound;

  function windSpeed
    input SI.Position h;
    input SI.Angle[:] vec_angles = wind_direction;
    input SI.Velocity[:] vec_magnitudes = wind_magnitude;
    input SI.Length[:] vec_heights = wind_layer_height;
    output SI.Velocity[3] windSpeed;
    //  output SI.Velocity mag;
    //  output SI.Angle dir;
  protected
    SI.Length cum_height = 0;
  algorithm
    for i in 1:size(vec_angles, 1) loop
      cum_height := cum_height + vec_heights[i];
      if h - world.altitude_0 <= cum_height then
        windSpeed := vec_magnitudes[i] * {cos(vec_angles[i]), sin(vec_angles[i]), 0};
        return;
      end if;
    end for;
//      mag := vec_magnitudes[i];
//      dir := vec_angles[i];
    if h - world.altitude_0 > cum_height then
      windSpeed := vec_magnitudes[end] * {cos(vec_angles[end]), sin(vec_angles[end]), 0};
    end if;
//    mag := vec_magnitudes[end];
//      dir := vec_angles[end];
//  annotation(
//    Inline = true);
  end windSpeed;

  package Blocks
    model Density
      extends Icon;
      outer RocketControl.World.Atmosphere atmosphere;
      Modelica.Blocks.Interfaces.RealInput h annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput rho annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      rho = atmosphere.density(h);
      annotation(
        Icon(graphics = {Text(origin = {1, 11}, extent = {{-65, 67}, {65, -67}}, textString = "rho"), Text(origin = {-121, -42}, textColor = {102, 102, 102}, extent = {{-21, 22}, {21, -22}}, textString = "h"), Text(origin = {0, -223}, textColor = {0, 0, 255}, extent = {{-119, 124}, {119, 84}}, textString = "%name")}));
    end Density;

    model Icon
    equation

      annotation(
        Icon(coordinateSystem(grid = {2, 0}), graphics = {Rectangle(fillColor = {228, 251, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20)}));
    end Icon;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Blocks;

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0), iconTransformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0)));

  model TurbolentWindSpeed
    parameter SI.Length wing_span;
    parameter SI.Duration wind_noise_sample_period = 0.05;
    parameter Integer wind_q_sign = 1;
    parameter Integer wind_r_sign = 1;
    parameter Integer wind_severity_high(min = 0, max = 6) = 1;
    import Modelica.Mechanics.MultiBody.Frames;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(visible = true, transformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0), iconTransformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput v_wind[3](each unit = "m/s", each quantity = "Velocity") annotation(
      Placement(visible = true, transformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    WindTurbolence windTurbolence(b = wing_span, noise_sample_period = wind_noise_sample_period, q_sign = wind_q_sign, r_sign = wind_r_sign, severity = wind_severity_high) annotation(
      Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput w_wind[3](each unit = "rad/s", each quantity = "AngularVelocity", each displayUnit = "deg/s") annotation(
      Placement(visible = true, transformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    BaseWindSpeed baseWindSpeed annotation(
      Placement(visible = true, transformation(origin = {-58, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    SI.Velocity v_wind_world[3];
    SI.Velocity v_turb_world[3];
    SI.Velocity w_wind_world[3];
    SI.Angle wind_dir(displayUnit = "deg");
    SI.Velocity wind_mag;
    Real A_nw[3, 3];
  equation
    A_nw = [cos(baseWindSpeed.dir), -sin(baseWindSpeed.dir), 0; sin(baseWindSpeed.dir), cos(baseWindSpeed.dir), 0; 0, 0, 1];
    v_turb_world = A_nw * windTurbolence.v_turb;
    v_wind_world = baseWindSpeed.v_wind + v_turb_world;
    w_wind_world = A_nw * windTurbolence.w_turb;
    v_wind = Frames.resolve2(frame_a.R, v_wind_world);
    w_wind = Frames.resolve2(frame_a.R, w_wind_world);
//  windTurbolence.V = wind_mag;
    wind_dir = atan2(v_wind_world[2], v_wind_world[1]);
    wind_mag = norm(v_wind_world);
    connect(frame_a, windTurbolence.frame_a) annotation(
      Line(points = {{-100, 0}, {-20, 0}}));
    connect(baseWindSpeed.v_wind, windTurbolence.v_wind) annotation(
      Line(points = {{-46, 20}, {-30, 20}, {-30, -8}, {-22, -8}}, color = {0, 0, 127}, thickness = 0.5));
    connect(frame_a, baseWindSpeed.frame_a) annotation(
      Line(points = {{-100, 0}, {-80, 0}, {-80, 20}, {-68, 20}}));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end TurbolentWindSpeed;

  model WindTurbolence
    outer RocketControl.World.Interfaces.WorldBase world;
    outer RocketControl.World.SimOptions opt;
    import Modelica.Constants.pi;
    parameter SI.Velocity W20 = wind_magnitude[1];
    parameter SI.Length b;
    parameter SI.Duration noise_sample_period = 0.05;
    parameter Integer q_sign = 1;
    parameter Integer r_sign = 1;
    parameter Integer severity(min = 0, max = 6) = 1;
    Modelica.Blocks.Noise.BandLimitedWhiteNoise wn_u(enableNoise = wind_turbolence_enable, noisePower = pi, samplePeriod = noise_sample_period, useAutomaticLocalSeed = true, useGlobalSeed = true) annotation(
      Placement(visible = true, transformation(origin = {-50, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    input SI.Velocity V;
    Modelica.Blocks.Noise.BandLimitedWhiteNoise wn_v(enableNoise = wind_turbolence_enable, noisePower = pi, samplePeriod = noise_sample_period, useAutomaticLocalSeed = true, useGlobalSeed = true) annotation(
      Placement(visible = true, transformation(origin = {-50, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Noise.BandLimitedWhiteNoise wn_w(enableNoise = wind_turbolence_enable, noisePower = pi, samplePeriod = noise_sample_period, useAutomaticLocalSeed = true, useGlobalSeed = true) annotation(
      Placement(visible = true, transformation(origin = {-50, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Blocks.Noise.BandLimitedWhiteNoise wn_p(enableNoise = wind_turbolence_enable, fixedLocalSeed = 1233142, noisePower = pi, samplePeriod = noise_sample_period, useAutomaticLocalSeed = false, useGlobalSeed = false) annotation(
      Placement(visible = true, transformation(origin = {-50, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput v_turb[3] annotation(
      Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput w_turb[3] annotation(
      Placement(visible = true, transformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_wind[3] annotation(
      Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Tables.CombiTable2Ds sigma_sev(extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints, table = [0,0,1,2,3,4,5,6;0.000,3.182,4.226,6.646,8.560,11.883,15.604,18.746;2041.399,2.077,3.523,7.018,9.662,13.140,17.663,21.899;3805.402,1.421,3.340,7.340,10.514,15.810,23.044,28.434;7573.307,0.000,1.551,6.710,10.119,15.026,23.622,30.240;25070.916,0.000,0.000,2.619,6.521,9.812,20.046,31.007;35135.591,0.000,0.000,0.372,5.026,8.129,15.929,25.182;44876.188,0.000,0.000,0.007,4.254,8.167,15.134,23.211;54937.621,0.000,0.000,0.000,2.692,7.910,12.100,17.538;65107.441,0.000,0.000,0.000,0.002,4.932,7.838,10.701;74221.927,0.000,0.000,0.000,0.000,3.412,6.321,8.570;80000.000,0.000,0.000,0.000,0.000,2.130,5.104,7.219]) annotation(
      Placement(visible = true, transformation(origin = {10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant sev(k = severity) annotation(
      Placement(visible = true, transformation(origin = {-34, 84}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    TurbolenceTF turb_low(b_ft = b_ft, q_sign = q_sign, r_sign = r_sign) annotation(
      Placement(visible = true, transformation(origin = {30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TurbolenceTF turb_high(b_ft = b_ft, q_sign = q_sign, r_sign = r_sign) annotation(
      Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    final parameter Real b_ft = b / 0.3048;
    final parameter Real W20_ft = W20 / 0.3048;
    SI.Position h;
    SI.Position h_low;
  equation
    V = 50 / 0.3048;
  
//max(norm(der(frame_a.r_0)) / 0.3048, 20);
  //  h = 1500 / 0.3048;
    h = max(world.altitude_agl(frame_a.r_0) / 0.3048, 1 / 0.3048);
    h_low = 750;
  //h_low = min(h, 1000);
    sigma_sev.u1 = 1500/0.3048;
    turb_low.L[3] = h_low / 2;
    turb_low.L[1] = h_low / (0.177 + 0.000823 * h_low) ^ 1.2;
    turb_low.L[2] = turb_low.L[1] / 2;
    turb_low.sigma[3] = 0.1 * W20_ft;
    turb_low.sigma[1] = turb_low.sigma[3] / (0.177 + 0.000823 * h_low) ^ 0.4;
    turb_low.sigma[2] = turb_low.sigma[1];
    turb_low.V = V;
    turb_high.L[1] = 1750;
    turb_high.L[2] = 1750 / 2;
    turb_high.L[3] = 1750 / 2;
    turb_high.sigma[1] = sigma_sev.y;
    turb_high.sigma[2] = sigma_sev.y;
    turb_high.sigma[3] = sigma_sev.y;
    turb_high.V = V;
    if time > 0.5 then
      if h < 1000 then
        v_turb = turb_low.v_turb;
        w_turb = turb_low.w_turb;
      elseif h > 2000 then
        v_turb = turb_high.v_turb;
        w_turb = turb_high.w_turb;
      else
        v_turb = turb_low.v_turb + (turb_high.v_turb - turb_low.v_turb) * (h - 1000) / 1000;
        w_turb = turb_low.w_turb + (turb_high.w_turb - turb_low.w_turb) * (h - 1000) / 1000;
      end if;
    else
      v_turb = zeros(3);
      w_turb = zeros(3);
    end if;
    frame_a.f = zeros(3);
    frame_a.t = zeros(3);
    connect(sev.y, sigma_sev.u2) annotation(
      Line(points = {{-27.4, 84}, {-2.4, 84}}, color = {0, 0, 127}));
    connect(wn_u.y, turb_low.noise[1]) annotation(
      Line(points = {{-38, 44}, {-20, 44}, {-20, 30}, {18, 30}}, color = {0, 0, 127}));
    connect(wn_v.y, turb_low.noise[2]) annotation(
      Line(points = {{-38, 14}, {-20, 14}, {-20, 30}, {18, 30}}, color = {0, 0, 127}));
    connect(wn_w.y, turb_low.noise[3]) annotation(
      Line(points = {{-38, -16}, {-20, -16}, {-20, 30}, {18, 30}}, color = {0, 0, 127}));
    connect(wn_p.y, turb_low.noise[4]) annotation(
      Line(points = {{-38, -46}, {-20, -46}, {-20, 30}, {18, 30}}, color = {0, 0, 127}));
    connect(wn_u.y, turb_high.noise[1]) annotation(
      Line(points = {{-38, 44}, {-20, 44}, {-20, -30}, {18, -30}}, color = {0, 0, 127}));
    connect(wn_v.y, turb_high.noise[2]) annotation(
      Line(points = {{-38, 14}, {-20, 14}, {-20, -30}, {18, -30}}, color = {0, 0, 127}));
    connect(wn_w.y, turb_high.noise[3]) annotation(
      Line(points = {{-38, -16}, {-20, -16}, {-20, -30}, {18, -30}}, color = {0, 0, 127}));
    connect(wn_p.y, turb_high.noise[4]) annotation(
      Line(points = {{-38, -46}, {-20, -46}, {-20, -30}, {18, -30}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end WindTurbolence;

  model BaseWindSpeed
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(visible = true, transformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0), iconTransformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput v_wind[3](each unit = "m/s", each quantity = "Velocity") annotation(
      Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput dir(unit = "rad", quantity = "Angle", displayUnit = "deg") annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput mag(unit = "m/s", quantity = "Velocity") annotation(
      Placement(visible = true, transformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    SI.Position h;
  equation
    h = world.altitude(frame_a.r_0);
    v_wind = windSpeed(h);
    mag = norm(v_wind);
    dir = atan2(v_wind[2], v_wind[1]);
    frame_a.f = zeros(3);
    frame_a.t = zeros(3);
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end BaseWindSpeed;

  TurbolentWindSpeed turb_wind_speed(wind_noise_sample_period = wind_noise_sample_period, wind_q_sign = wind_q_sign, wind_r_sign = wind_r_sign, wind_severity_high = wind_severity_high, wing_span = wing_span) annotation(
    Placement(visible = true, transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  model TurbolenceTF
    import Modelica.Constants.pi;
    parameter Real b_ft;
    parameter Integer q_sign = 1;
    parameter Integer r_sign = 1;
    RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hv(n = 2) annotation(
      Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hp(n = 1) annotation(
      Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput w_turb[3] annotation(
      Placement(visible = true, transformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hu(n = 1) annotation(
      Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hw(n = 2) annotation(
      Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hq(n = 1) annotation(
      Placement(visible = true, transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Blocks.Math.ParameterVaryingTransferFunction Hr(n = 1) annotation(
      Placement(visible = true, transformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput v_turb[3] annotation(
      Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput L[3] annotation(
      Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput sigma[3] annotation(
      Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput noise[4] annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    input SI.Velocity V;
    RocketControl.Blocks.Trasformations.Units.ft2m ft2m(n = 3) annotation(
      Placement(visible = true, transformation(origin = {70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
// Velocity
    Hu.b = sigma[1] * sqrt(2 * L[1] / (pi * V)) * {0, 1};
    Hu.a = {L[1] / V, 1};
    Hv.b = sigma[2] * sqrt(2 * L[2] / (pi * V)) * {0, 2 * sqrt(3) * L[2] / V, 1};
    Hv.a = {(2 * L[2] / V) ^ 2, 2 * (2 * L[2] / V), 1};
    Hw.b = sigma[3] * sqrt(2 * L[3] / (pi * V)) * {0, 2 * sqrt(3) * L[3] / V, 1};
    Hw.a = {(2 * L[3] / V) ^ 2, 2 * (2 * L[3] / V), 1};
// Angular Velocity
    Hp.b = sigma[3] * sqrt(0.8 / V) * {0, (pi / (4 * b_ft)) ^ (1 / 6)};
    Hp.a = (2 * L[3]) ^ (1 / 3) * {4 * b_ft / pi, 1};
    Hq.b = {q_sign * 1 / V, 0};
    Hq.a = {4 * b_ft / (pi * V), 1};
    Hr.b = {r_sign * 1 / V, 0};
    Hr.a = {3 * b_ft / (pi * V), 1};
    w_turb[1] = min(max(Hp.y, -1), 1);
    w_turb[2] = min(max(Hq.y, -1), 1);
    w_turb[3] = min(max(Hr.y, -1), 1);
    connect(Hw.y, Hq.u) annotation(
      Line(points = {{12, 30}, {20, 30}, {20, -30}, {-20, -30}, {-20, -50}, {-12, -50}}, color = {0, 0, 127}));
    connect(Hv.y, Hr.u) annotation(
      Line(points = {{12, 60}, {22, 60}, {22, -70}, {-20, -70}, {-20, -90}, {-12, -90}}, color = {0, 0, 127}));
    connect(noise[1], Hu.u) annotation(
      Line(points = {{-120, 0}, {-60, 0}, {-60, 90}, {-12, 90}}, color = {0, 0, 127}));
    connect(noise[2], Hv.u) annotation(
      Line(points = {{-120, 0}, {-60, 0}, {-60, 60}, {-12, 60}}, color = {0, 0, 127}));
    connect(noise[3], Hw.u) annotation(
      Line(points = {{-120, 0}, {-60, 0}, {-60, 30}, {-12, 30}}, color = {0, 0, 127}));
    connect(noise[4], Hp.u) annotation(
      Line(points = {{-120, 0}, {-60, 0}, {-60, -10}, {-12, -10}}, color = {0, 0, 127}));
    connect(ft2m.m, v_turb) annotation(
      Line(points = {{82, 60}, {110, 60}}, color = {0, 0, 127}, thickness = 0.5));
    connect(Hu.y, ft2m.ft[1]) annotation(
      Line(points = {{12, 90}, {40, 90}, {40, 60}, {58, 60}}, color = {0, 0, 127}));
    connect(Hv.y, ft2m.ft[2]) annotation(
      Line(points = {{12, 60}, {58, 60}}, color = {0, 0, 127}));
    connect(Hw.y, ft2m.ft[3]) annotation(
      Line(points = {{12, 30}, {40, 30}, {40, 60}, {58, 60}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end TurbolenceTF;
equation
  connect(frame_a, turb_wind_speed.frame_a) annotation(
    Line(points = {{-100, 0}, {-8, 0}}));
  annotation(
    defaultComponentPrefixes = "inner",
    defaultComponentName = "atmosphere",
    Icon(graphics = {Rectangle(fillColor = {202, 244, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Ellipse(origin = {-53, -46}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {-19, 0}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {-3, -58}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {61, -44}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {43, -10}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {-65, 56}, lineColor = {255, 255, 255}, fillColor = {255, 255, 0}, fillPattern = FillPattern.Sphere, extent = {{-21, 20}, {21, -20}}), Line(origin = {-63.08, 24.12}, points = {{6, 9}, {12, -9}}, color = {255, 255, 0}, arrowSize = 1), Line(origin = {-85.33, 26.82}, points = {{6, 9}, {-6, -9}}, color = {255, 255, 0}, arrowSize = 1), Line(origin = {-74.2, 23.11}, points = {{6, 9}, {2, -9}}, color = {255, 255, 0}, arrowSize = 1), Line(origin = {-44.2, 45.36}, points = {{6, 9}, {24, 5}}, color = {255, 255, 0}), Line(origin = {-49.26, 30.53}, points = {{6, 9}, {18, -5}}, color = {255, 255, 0}), Line(origin = {-46.56, 59.51}, points = {{6, 9}, {26, 11}}, color = {255, 255, 0}), Text(origin = {4, 20}, lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name")}));
end Atmosphere;
