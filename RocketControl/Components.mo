within RocketControl;

package Components
  package Sensors
    model AeroAnglesSensor
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      import Modelica.Mechanics.MultiBody.Frames;
      import Modelica.Math.Vectors;
      outer World.Atmosphere atmosphere;
      Modelica.Blocks.Interfaces.RealOutput aoa(final quantity = "Angle", final unit = "rad", displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {102, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Velocity v_b[3] "Velocity relative to wind in body frame (body frame)";
      SI.Velocity v_w[3] "Wind speed at current position";
      SI.Velocity v_norm;
      parameter SI.Velocity v_small = 1e-1 "Prevent division by zero when velocity is too small";
      Modelica.Blocks.Interfaces.RealOutput sideslip(final quantity = "Angle", final unit = "rad", displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      v_w = atmosphere.windSpeed(frame_a.r_0);
      v_b = Frames.resolve2(frame_a.R, der(frame_a.r_0) - v_w);
      v_norm = Vectors.norm(v_b);
  if abs(v_b[1]) > v_small then
//aoa = atan(sqrt(v_b[3]^2 + v_b[2]^2)/v_b[1]);
        aoa = atan(v_b[3] / v_b[1]);
//aoa = atan2(v_b[3], v_b[1]);
      elseif abs(v_b[3]) > v_small then
        aoa = Modelica.Units.Conversions.from_deg(90 * sign(v_b[3]));
      else
        aoa = 0;
      end if;
  if abs(v_b[1]) > v_small then
        sideslip = atan(v_b[2] / v_b[1]);
//sideslip = asin(v_b[2] / v_norm);
      elseif abs(v_b[2]) > v_small then
        sideslip = Modelica.Units.Conversions.from_deg(90 * sign(v_b[2]));
      else
        sideslip = 0;
      end if;
      frame_a.f = zeros(3);
      frame_a.t = zeros(3);
      annotation(
        Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Text(origin = {-10, 51}, extent = {{-90, 19}, {90, -19}}, textString = "AoA", horizontalAlignment = TextAlignment.Right), Text(origin = {-8, -49}, extent = {{-90, 19}, {90, -19}}, textString = "beta", horizontalAlignment = TextAlignment.Right)}));
    end AeroAnglesSensor;

    model AeroStateSensor
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      import Modelica.Mechanics.MultiBody.Frames;
      import Modelica.Math.Vectors;
      RocketControl.Components.Sensors.AeroAnglesSensor aeroAnglesSensor annotation(
        Placement(visible = true, transformation(origin = {0, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      outer World.Atmosphere atmosphere;
      outer World.MyWorld world;
      Modelica.Units.SI.Angle betaprime;
      SI.Velocity[3] v_w;
      SI.Velocity[3] v;
      RocketControl.Interfaces.AeroStateOutput aeroStateOutput annotation(
        Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
        Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      v_w = atmosphere.windSpeed(frame_a.r_0);
      aeroStateOutput.alpha = aeroAnglesSensor.aoa;
      aeroStateOutput.beta = aeroAnglesSensor.sideslip;
      aeroStateOutput.mach = Vectors.norm(der(frame_a.r_0)) / atmosphere.speedOfSound(frame_a.r_0);
      aeroStateOutput.altitude = world.altitude(frame_a.r_0);
      v = Frames.resolve2(frame_a.R, der(frame_a.r_0) - v_w);
      aeroStateOutput.v = v;
      aeroStateOutput.w = absoluteAngularVelocity.w;
      betaprime = atan2(v[2], v[1]);
      connect(aeroAnglesSensor.frame_a, frame_a) annotation(
        Line(points = {{-10, 34}, {-55, 34}, {-55, 0}, {-100, 0}}));
      connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
        Line(points = {{-100, 0}, {-54, 0}, {-54, -40}, {-10, -40}}));
      annotation(
        Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Text(origin = {1, -40}, lineColor = {64, 64, 64}, extent = {{-101, 20}, {101, -20}}, textString = "aero"), Line(origin = {84.1708, -2.17082}, points = {{-14.1708, 2.17082}, {11.8292, 2.17082}, {13.8292, -1.82918}})}));
    end AeroStateSensor;

    model LaunchRailPresenceSensor "Detects wheter frame_lug is within a distance of frame_rail along a certain direction"
      import Modelica.Mechanics.MultiBody.Frames.resolve2;
      parameter Real n[3] = {0, 0, 1};
      parameter SI.Length rail_length;
      SI.Position s[3];
      SI.Length d;
      Modelica.Blocks.Interfaces.BooleanOutput y annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lug annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -38}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_rail annotation(
        Placement(visible = true, transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
    initial equation
      y = true;
    equation
      s = resolve2(frame_rail.R, frame_lug.r_0 - frame_rail.r_0);
      d = s * n;
      when d > rail_length then
        y = false;
      end when;
      frame_rail.f = zeros(3);
      frame_rail.t = zeros(3);
      frame_lug.f = zeros(3);
      frame_lug.t = zeros(3);
      annotation(
        Icon(graphics = {Polygon(origin = {-9, 8}, fillColor = {222, 222, 222}, fillPattern = FillPattern.Forward, points = {{17, 88}, {-71, -78}, {-71, -88}, {71, -88}, {49, -80}, {-55, -80}, {17, 62}, {17, 88}}), Polygon(origin = {-2, 30}, fillColor = {85, 122, 162}, fillPattern = FillPattern.VerticalCylinder, points = {{-28, -42}, {-14, -50}, {-14, -42}, {26, 38}, {28, 50}, {20, 42}, {-20, -38}, {-28, -42}}), Line(origin = {-35, -41}, points = {{11, 21}, {-11, -23}}, thickness = 0.75), Line(origin = {-27.06, -24.0629}, points = {{-4.93552, 0.159125}, {3.06448, 4.15913}, {5.06448, -3.84087}, {5.06448, -3.84087}}, thickness = 0.75), Line(origin = {-43.4078, -60.2878}, points = {{-5, 3.87873}, {-3, -4.12127}, {5, -2.12127}, {5, -2.12127}}, thickness = 0.75), Text(origin = {-566.66, 40}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "lug"), Text(origin = {-620.66, -52}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "rail"), Text(origin = {0, 8}, lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name")}));
    end LaunchRailPresenceSensor;

    model AssetSensor
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      import Modelica.Mechanics.MultiBody.Frames;
      import Modelica.Units.Conversions.to_deg;
      Real x_b[3];
      Real y_b[3];
      Real z_b[3];
      Real yaw_den;
      Modelica.Blocks.Interfaces.RealOutput yaw annotation(
        Placement(visible = true, transformation(origin = {106, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput pitch annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput roll annotation(
        Placement(visible = true, transformation(origin = {106, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      x_b = frame_a.R.T[1, :];
      y_b = frame_a.R.T[:, 2];
      z_b = frame_a.R.T[:, 3];
      yaw_den = x_b * {1, 0, 0};
      if abs(yaw_den) > 1e-7 then
        yaw = to_deg(atan(x_b * {0, 1, 0} / yaw_den));
      else
        yaw = 0;
      end if;
      pitch = to_deg(asin(x_b * {0, 0, -1}));
      roll = 0;
      frame_a.f = zeros(3);
      frame_a.t = zeros(3);
      annotation(
        Icon(graphics = {Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "ypr"), Line(origin = {81, 30}, points = {{-11, -30}, {-1, -30}, {-1, 30}, {11, 30}}), Line(origin = {86, 0}, points = {{-6, 0}, {6, 0}}), Line(origin = {86, -30}, points = {{-6, 30}, {-6, -30}, {6, -30}}), Text(origin = {322, 54}, lineColor = {128, 128, 128}, extent = {{-270, 43}, {-180, 18}}, textString = "yaw"), Text(origin = {328, -4}, lineColor = {128, 128, 128}, extent = {{-276, 43}, {-184, 18}}, textString = "pitch"), Text(origin = {324, -68}, lineColor = {128, 128, 128}, extent = {{-270, 43}, {-180, 18}}, textString = "roll")}));
    end AssetSensor;

    model RealGyroscope "Implementation of a real gyroscope, affected by startup random bias, bias instability (Rate Random Walk) and Normal Noise (Angle Random Walk)"
      import Modelica.Units.Conversions.from_deg;
      parameter SI.Frequency samplingRate;
      parameter SI.AngularVelocity bias[3];
      parameter SI.Duration noiseSamplePeriod "Noise sample period";
      parameter SI.AngularVelocity sigmaARW;
      parameter SI.AngularAcceleration sigmaRRW;
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput w_meas[3] annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Noise.NormalNoiseAndRandomWalk noiseX(samplePeriod = noiseSamplePeriod, sigmaNoise = sigmaARW, sigmaRW = sigmaRRW) annotation(
        Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum multiSumX(nu = 3) annotation(
        Placement(visible = true, transformation(origin = {64, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant biasX(k = bias[1]) annotation(
        Placement(visible = true, transformation(origin = {30, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Noise.NormalNoiseAndRandomWalk noiseY(samplePeriod = noiseSamplePeriod, sigmaNoise = sigmaARW, sigmaRW = sigmaRRW) annotation(
        Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant biasY(k = bias[2]) annotation(
        Placement(visible = true, transformation(origin = {30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum multiSum(nu = 3) annotation(
        Placement(visible = true, transformation(origin = {64, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant biasZ(k = bias[3]) annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Noise.NormalNoiseAndRandomWalk noiseZ(samplePeriod = noiseSamplePeriod, sigmaNoise = sigmaARW, sigmaRW = sigmaRRW) annotation(
        Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum multiSum1(nu = 3) annotation(
        Placement(visible = true, transformation(origin = {64, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
        Line(points = {{-100, 0}, {-80, 0}}));
      connect(biasX.y, multiSumX.u[1]) annotation(
        Line(points = {{42, 90}, {48, 90}, {48, 60}, {58, 60}}, color = {0, 0, 127}));
      connect(noiseX.y, multiSumX.u[2]) annotation(
        Line(points = {{2, 90}, {12, 90}, {12, 60}, {58, 60}}, color = {0, 0, 127}));
      connect(noiseY.y, multiSum.u[2]) annotation(
        Line(points = {{1, 30}, {11, 30}, {11, 0}, {57, 0}}, color = {0, 0, 127}));
      connect(biasY.y, multiSum.u[1]) annotation(
        Line(points = {{41, 30}, {47, 30}, {47, 0}, {57, 0}}, color = {0, 0, 127}));
      connect(biasZ.y, multiSum1.u[1]) annotation(
        Line(points = {{41, -30}, {47, -30}, {47, -60}, {58, -60}}, color = {0, 0, 127}));
      connect(noiseZ.y, multiSum1.u[2]) annotation(
        Line(points = {{1, -30}, {11, -30}, {11, -60}, {58, -60}}, color = {0, 0, 127}));
      connect(multiSumX.y, w_meas[1]) annotation(
        Line(points = {{72, 60}, {80, 60}, {80, 0}, {106, 0}}, color = {0, 0, 127}));
      connect(multiSum.y, w_meas[2]) annotation(
        Line(points = {{72, 0}, {106, 0}}, color = {0, 0, 127}));
      connect(multiSum1.y, w_meas[3]) annotation(
        Line(points = {{72, -60}, {80, -60}, {80, 0}, {106, 0}}, color = {0, 0, 127}));
      connect(absoluteAngularVelocity.w[1], multiSumX.u[3]) annotation(
        Line(points = {{-58, 0}, {-40, 0}, {-40, 60}, {58, 60}}, color = {0, 0, 127}));
      connect(absoluteAngularVelocity.w[2], multiSum.u[3]) annotation(
        Line(points = {{-58, 0}, {58, 0}}, color = {0, 0, 127}));
      connect(absoluteAngularVelocity.w[3], multiSum1.u[3]) annotation(
        Line(points = {{-58, 0}, {-40, 0}, {-40, -60}, {58, -60}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "rad/s")}),
        Diagram);
    end RealGyroscope;

    model PartialRealInertialSensor
      parameter SI.Frequency samplingRate;
      parameter Real bias[3] "Fixed bias";
      parameter SI.Duration noiseSamplePeriod "Noise sample period";
      parameter Real sigmaNoise "Standard deviation of the white noise affecting the sensor";
      parameter Real sigmaBiasStability "Standard deviation of the noise generating the bias instability random walk";
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      Modelica.Blocks.Interfaces.RealOutput w_meas[3] annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Noise.NormalNoiseAndRandomWalk noiseX(samplePeriod = noiseSamplePeriod, sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasStability) annotation(
        Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum multiSumX(nu = 2) annotation(
        Placement(visible = true, transformation(origin = {64, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant biasX(k = bias[1]) annotation(
        Placement(visible = true, transformation(origin = {30, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Noise.NormalNoiseAndRandomWalk noiseY(samplePeriod = noiseSamplePeriod, sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasStability) annotation(
        Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant biasY(k = bias[2]) annotation(
        Placement(visible = true, transformation(origin = {30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum multiSum(nu = 2) annotation(
        Placement(visible = true, transformation(origin = {64, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant biasZ(k = bias[3]) annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Noise.NormalNoiseAndRandomWalk noiseZ(samplePeriod = noiseSamplePeriod, sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasStability) annotation(
        Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum multiSum1(nu = 2) annotation(
        Placement(visible = true, transformation(origin = {64, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      connect(biasX.y, multiSumX.u[1]) annotation(
        Line(points = {{42, 90}, {48, 90}, {48, 60}, {58, 60}}, color = {0, 0, 127}));
      connect(noiseX.y, multiSumX.u[2]) annotation(
        Line(points = {{2, 90}, {12, 90}, {12, 60}, {58, 60}}, color = {0, 0, 127}));
      connect(noiseY.y, multiSum.u[2]) annotation(
        Line(points = {{1, 30}, {11, 30}, {11, 0}, {57, 0}}, color = {0, 0, 127}));
      connect(biasY.y, multiSum.u[1]) annotation(
        Line(points = {{41, 30}, {47, 30}, {47, 0}, {57, 0}}, color = {0, 0, 127}));
      connect(biasZ.y, multiSum1.u[1]) annotation(
        Line(points = {{41, -30}, {47, -30}, {47, -60}, {58, -60}}, color = {0, 0, 127}));
      connect(noiseZ.y, multiSum1.u[2]) annotation(
        Line(points = {{1, -30}, {11, -30}, {11, -60}, {58, -60}}, color = {0, 0, 127}));
      connect(multiSumX.y, w_meas[1]) annotation(
        Line(points = {{72, 60}, {80, 60}, {80, 0}, {106, 0}}, color = {0, 0, 127}));
      connect(multiSum.y, w_meas[2]) annotation(
        Line(points = {{72, 0}, {106, 0}}, color = {0, 0, 127}));
      connect(multiSum1.y, w_meas[3]) annotation(
        Line(points = {{72, -60}, {80, -60}, {80, 0}, {106, 0}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "rad/s")}),
        Diagram);
    end PartialRealInertialSensor;

    model RealAccelerometer
      parameter SI.Frequency samplingRate;
      parameter SI.Acceleration bias[3];
      parameter SI.Duration noiseSamplePeriod "Noise sample period";
      parameter SI.Acceleration sigmaNoise;
      parameter SI.Jerk sigmaBiasInstability;
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      Modelica.Blocks.Interfaces.RealOutput acc[3] annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Noise.NormalNoiseAndRandomWalk noiseX(samplePeriod = noiseSamplePeriod, sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasInstability) annotation(
        Placement(visible = true, transformation(origin = {-26, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum multiSumX(nu = 3) annotation(
        Placement(visible = true, transformation(origin = {48, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant biasX(k = bias[1]) annotation(
        Placement(visible = true, transformation(origin = {14, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Noise.NormalNoiseAndRandomWalk noiseY(samplePeriod = noiseSamplePeriod, sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasInstability) annotation(
        Placement(visible = true, transformation(origin = {-26, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant biasY(k = bias[2]) annotation(
        Placement(visible = true, transformation(origin = {14, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum multiSum(nu = 3) annotation(
        Placement(visible = true, transformation(origin = {48, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant biasZ(k = bias[3]) annotation(
        Placement(visible = true, transformation(origin = {14, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Noise.NormalNoiseAndRandomWalk noiseZ(samplePeriod = noiseSamplePeriod, sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasInstability) annotation(
        Placement(visible = true, transformation(origin = {-26, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.MultiSum multiSum1(nu = 3) annotation(
        Placement(visible = true, transformation(origin = {48, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      RocketControl.Components.Sensors.IdealAccelerometer idealAccelerometer annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Clocked.RealSignals.Sampler.SampleVectorizedAndClocked sample1(n = 3) annotation(
        Placement(visible = true, transformation(origin = {80, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Clocked.ClockSignals.Clocks.PeriodicRealClock periodicClock1(period = 1 / samplingRate) annotation(
        Placement(visible = true, transformation(origin = {62, -88}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      connect(biasX.y, multiSumX.u[1]) annotation(
        Line(points = {{25, 90}, {31, 90}, {31, 60}, {41, 60}}, color = {0, 0, 127}));
      connect(noiseX.y, multiSumX.u[2]) annotation(
        Line(points = {{-15, 90}, {-5, 90}, {-5, 60}, {41, 60}}, color = {0, 0, 127}));
      connect(noiseY.y, multiSum.u[2]) annotation(
        Line(points = {{-15, 30}, {-5, 30}, {-5, 0}, {41, 0}}, color = {0, 0, 127}));
      connect(biasY.y, multiSum.u[1]) annotation(
        Line(points = {{25, 30}, {31, 30}, {31, 0}, {41, 0}}, color = {0, 0, 127}));
      connect(biasZ.y, multiSum1.u[1]) annotation(
        Line(points = {{25, -30}, {31, -30}, {31, -60}, {42, -60}}, color = {0, 0, 127}));
      connect(noiseZ.y, multiSum1.u[2]) annotation(
        Line(points = {{-15, -30}, {-5, -30}, {-5, -60}, {42, -60}}, color = {0, 0, 127}));
      connect(frame_a, idealAccelerometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-80, 0}}));
      connect(idealAccelerometer.acc[1], multiSumX.u[3]) annotation(
        Line(points = {{-60, 0}, {-40, 0}, {-40, 60}, {42, 60}}, color = {0, 0, 127}));
      connect(idealAccelerometer.acc[2], multiSum.u[3]) annotation(
        Line(points = {{-60, 0}, {42, 0}}, color = {0, 0, 127}));
      connect(idealAccelerometer.acc[3], multiSum1.u[3]) annotation(
        Line(points = {{-60, 0}, {-40, 0}, {-40, -60}, {42, -60}}, color = {0, 0, 127}));
      connect(sample1.y, acc) annotation(
        Line(points = {{86, 0}, {106, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(multiSumX.y, sample1.u[1]) annotation(
        Line(points = {{56, 60}, {64, 60}, {64, 0}, {72, 0}}, color = {0, 0, 127}));
      connect(multiSum.y, sample1.u[2]) annotation(
        Line(points = {{56, 0}, {72, 0}}, color = {0, 0, 127}));
      connect(multiSum1.y, sample1.u[3]) annotation(
        Line(points = {{56, -60}, {64, -60}, {64, 0}, {72, 0}}, color = {0, 0, 127}));
      connect(periodicClock1.y, sample1.clock) annotation(
        Line(points = {{68, -88}, {80, -88}, {80, -8}}, color = {175, 175, 175}));
      annotation(
        Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "rad/s")}),
        Diagram);
    end RealAccelerometer;

    model IdealAccelerometer
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      outer World world;
      SI.Acceleration acc_inertial[3];
      SI.Acceleration acc_body[3];
      Modelica.Blocks.Interfaces.RealOutput acc[3] annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
        Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      acc_inertial = der(absoluteVelocity.v);
      acc_body = acc_inertial - world.gravityAcceleration(frame_a.r_0);
      acc =Modelica.Mechanics.MultiBody.Frames.resolve2(frame_a.R, acc_inertial);
      connect(frame_a, absoluteVelocity.frame_a) annotation(
        Line(points = {{-100, 0}, {-70, 0}}));
      annotation(
        Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "m/s^2")}));
    end IdealAccelerometer;
  end Sensors;

  model BodyVariableMass "Rigid body with mass, inertia tensor and one frame connector (12 potential states)"
    import Modelica.Mechanics.MultiBody.Visualizers;
    import Modelica.Mechanics.MultiBody.Types;
    import Modelica.Mechanics.MultiBody.Frames;
    import Modelica.Units.Conversions.to_unit1;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Coordinate system fixed at body" annotation(
      Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
    parameter Boolean animation = true "= true, if animation shall be enabled (show cylinder and sphere)";
    parameter SI.Position r_CM[3](start = {0, 0, 0}) "Vector from frame_a to center of mass, resolved in frame_a";
    SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a" annotation(
      Dialog(tab = "Initialization", showStartAttribute = true));
    SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(
      Dialog(tab = "Initialization", showStartAttribute = true));
    SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(
      Dialog(tab = "Initialization", showStartAttribute = true));
    parameter Boolean angles_fixed = false "= true, if angles_start are used as initial values, else as guess values" annotation(
      Evaluate = true,
      choices(checkBox = true),
      Dialog(tab = "Initialization"));
    parameter SI.Angle angles_start[3] = {0, 0, 0} "Initial values of angles to rotate world frame around 'sequence_start' axes into frame_a" annotation(
      Dialog(tab = "Initialization"));
    parameter Types.RotationSequence sequence_start = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a at initial time" annotation(
      Evaluate = true,
      Dialog(tab = "Initialization"));
    parameter Boolean w_0_fixed = false "= true, if w_0_start are used as initial values, else as guess values" annotation(
      Evaluate = true,
      choices(checkBox = true),
      Dialog(tab = "Initialization"));
    parameter SI.AngularVelocity w_0_start[3] = {0, 0, 0} "Initial or guess values of angular velocity of frame_a resolved in world frame" annotation(
      Dialog(tab = "Initialization"));
    parameter Boolean z_0_fixed = false "= true, if z_0_start are used as initial values, else as guess values" annotation(
      Evaluate = true,
      choices(checkBox = true),
      Dialog(tab = "Initialization"));
    parameter SI.AngularAcceleration z_0_start[3] = {0, 0, 0} "Initial values of angular acceleration z_0 = der(w_0)" annotation(
      Dialog(tab = "Initialization"));
    parameter SI.Diameter sphereDiameter = world.defaultBodyDiameter "Diameter of sphere" annotation(
      Dialog(tab = "Animation", group = "if animation = true", enable = animation));
    input Types.Color sphereColor = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of sphere" annotation(
      Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
    parameter SI.Diameter cylinderDiameter = sphereDiameter / Types.Defaults.BodyCylinderDiameterFraction "Diameter of cylinder" annotation(
      Dialog(tab = "Animation", group = "if animation = true", enable = animation));
    input Types.Color cylinderColor = sphereColor "Color of cylinder" annotation(
      Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
    input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(
      Dialog(tab = "Animation", group = "if animation = true", enable = animation));
    parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(
      Evaluate = true,
      Dialog(tab = "Advanced"));
    parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(
      Evaluate = true,
      Dialog(tab = "Advanced"));
    parameter Types.RotationSequence sequence_angleStates = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a around the 3 angles used as potential states" annotation(
      Evaluate = true,
      Dialog(tab = "Advanced", enable = not useQuaternions));
    SI.Mass m "Body Mass";
    SI.Inertia I[3, 3] "Inertia tensor";
    final parameter Frames.Orientation R_start = Modelica.Mechanics.MultiBody.Frames.axesRotations(sequence_start, angles_start, zeros(3)) "Orientation object from world frame to frame_a at initial time";
    SI.AngularVelocity w_a[3](start = Frames.resolve2(R_start, w_0_start), fixed = fill(w_0_fixed, 3), each stateSelect = if enforceStates then if useQuaternions then StateSelect.always else StateSelect.never else StateSelect.avoid) "Absolute angular velocity of frame_a resolved in frame_a";
    SI.AngularAcceleration z_a[3](fixed = fill(z_0_fixed, 3)) "Absolute angular acceleration of frame_a resolved in frame_a";
    SI.Acceleration g_0[3] "Gravity acceleration resolved in world frame";
    RocketControl.Interfaces.MassPropertiesInput massInput annotation(
      Placement(visible = true, transformation(origin = {0, -96}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {40, -62}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  protected
    outer Modelica.Mechanics.MultiBody.World world;
    // Declarations for quaternions (dummies, if quaternions are not used)
    parameter Frames.Quaternions.Orientation Q_start = Frames.to_Q(R_start) "Quaternion orientation object from world frame to frame_a at initial time";
    Frames.Quaternions.Orientation Q(start = Q_start, each stateSelect = if enforceStates then if useQuaternions then StateSelect.prefer else StateSelect.never else StateSelect.avoid) "Quaternion orientation object from world frame to frame_a (dummy value, if quaternions are not used as states)";
    // Declaration for 3 angles
    parameter SI.Angle phi_start[3] = if sequence_start[1] == sequence_angleStates[1] and sequence_start[2] == sequence_angleStates[2] and sequence_start[3] == sequence_angleStates[3] then angles_start else Frames.axesRotationsAngles(R_start, sequence_angleStates) "Potential angle states at initial time";
    SI.Angle phi[3](start = phi_start, each stateSelect = if enforceStates then if useQuaternions then StateSelect.never else StateSelect.always else StateSelect.avoid) "Dummy or 3 angles to rotate world frame into frame_a of body";
    SI.AngularVelocity phi_d[3](each stateSelect = if enforceStates then if useQuaternions then StateSelect.never else StateSelect.always else StateSelect.avoid) "= der(phi)";
    SI.AngularAcceleration phi_dd[3] "= der(phi_d)";
    // Declarations for animation
    Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = cylinderColor, specularCoefficient = specularCoefficient, length = if Modelica.Math.Vectors.length(r_CM) > sphereDiameter / 2 then Modelica.Math.Vectors.length(r_CM) - (if cylinderDiameter > 1.1 * sphereDiameter then sphereDiameter / 2 else 0) else 0, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = to_unit1(r_CM), widthDirection = {0, 1, 0}, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
    Visualizers.Advanced.Shape sphere(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = r_CM - {1, 0, 0} * sphereDiameter / 2, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation and sphereDiameter > 0;
  initial equation
    if angles_fixed then
// Initialize positional variables
      if not Connections.isRoot(frame_a.R) then
// frame_a.R is computed somewhere else
        zeros(3) = Frames.Orientation.equalityConstraint(frame_a.R, R_start);
      elseif useQuaternions then
// frame_a.R is computed from quaternions Q
        zeros(3) = Frames.Quaternions.Orientation.equalityConstraint(Q, Q_start);
      else
// frame_a.R is computed from the 3 angles 'phi'
        phi = phi_start;
      end if;
    end if;
  equation
    if enforceStates then
      Connections.root(frame_a.R);
    else
      Connections.potentialRoot(frame_a.R);
    end if;
    r_0 = frame_a.r_0;
  if not Connections.isRoot(frame_a.R) then
// Body does not have states
// Dummies
      Q = {0, 0, 0, 1};
      phi = zeros(3);
      phi_d = zeros(3);
      phi_dd = zeros(3);
    elseif useQuaternions then
// Use Quaternions as states (with dynamic state selection)
      frame_a.R = Frames.from_Q(Q, Frames.Quaternions.angularVelocity2(Q, der(Q)));
      {0} = Frames.Quaternions.orientationConstraint(Q);
// Dummies
      phi = zeros(3);
      phi_d = zeros(3);
      phi_dd = zeros(3);
    else
// Use Cardan angles as states
      phi_d = der(phi);
      phi_dd = der(phi_d);
      frame_a.R = Frames.axesRotations(sequence_angleStates, phi, phi_d);
// Dummies
      Q = {0, 0, 0, 1};
    end if;
// gravity acceleration at center of mass resolved in world frame
    g_0 = world.gravityAcceleration(frame_a.r_0 + Frames.resolve1(frame_a.R, r_CM));
    m = massInput.m;
    I = massInput.I;
// translational kinematic differential equations
    v_0 = der(frame_a.r_0);
    a_0 = der(v_0);
// rotational kinematic differential equations
    w_a = Frames.angularVelocity2(frame_a.R);
    z_a = der(w_a);
/* Newton/Euler equations with respect to center of mass
              a_CM = a_a + cross(z_a, r_CM) + cross(w_a, cross(w_a, r_CM));
              f_CM = m*(a_CM - g_a);
              t_CM = I*z_a + cross(w_a, I*w_a);
         frame_a.f = f_CM
         frame_a.t = t_CM + cross(r_CM, f_CM);
      Inserting the first three equations in the last two results in:
    */
    frame_a.f = m * (Frames.resolve2(frame_a.R, a_0 - g_0) + cross(z_a, r_CM) + cross(w_a, cross(w_a, r_CM)));
    frame_a.t = I * z_a + cross(w_a, I * w_a) + cross(r_CM, frame_a.f);
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 24, 48}, fillColor = {0, 127, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, 30}, {-3, -30}}, radius = 10), Text(lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Ellipse(lineColor = {0, 24, 48}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Sphere, extent = {{-20, 60}, {100, -60}}, endAngle = 360)}),
      Documentation(info = "<html>
  <p>
  <strong>Rigid body</strong> with mass and inertia tensor.
  All parameter vectors have to be resolved in frame_a.
  The <strong>inertia tensor</strong> has to be defined with respect to a
  coordinate system that is parallel to frame_a with the
  origin at the center of mass of the body.
  </p>
  <p>
  By default, this component is visualized by a <strong>cylinder</strong> located
  between frame_a and the center of mass and by a <strong>sphere</strong> that has
  its center at the center of mass. If the cylinder length is smaller as
  the radius of the sphere, e.g., since frame_a is located at the
  center of mass, the cylinder is not displayed. Note, that
  the animation may be switched off via parameter animation = <strong>false</strong>.
  </p>
  <p>
  <img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Parts/Body.png\" alt=\"Parts.Body\">
  </p>
  
  <p>
  <strong>States of Body Components</strong>
  </p>
  <p>
  Every body has potential states. If possible a tool will select
  the states of joints and not the states of bodies because this is
  usually the most efficient choice. In this case the position, orientation,
  velocity and angular velocity of frame_a of the body will be computed
  by the component that is connected to frame_a. However, if a body is moving
  freely in space, variables of the body have to be used as states. The potential
  states of the body are:
  </p>
  <ul>
  <li> The <strong>position vector</strong> frame_a.r_0 from the origin of the
       world frame to the origin of frame_a of the body, resolved in
       the world frame and the <strong>absolute velocity</strong> v_0 of the origin of
       frame_a, resolved in the world frame (= der(frame_a.r_0)).
  </li>
  <li> If parameter <strong>useQuaternions</strong> in the \"Advanced\" menu
       is <strong>true</strong> (this is the default), then <strong>4 quaternions</strong>
       are potential states. Additionally, the coordinates of the
       absolute angular velocity vector of the
       body are 3 potential states.<br>
       If <strong>useQuaternions</strong> in the \"Advanced\" menu
       is <strong>false</strong>, then <strong>3 angles</strong> and the derivatives of
       these angles are potential states. The orientation of frame_a
       is computed by rotating the world frame along the axes defined
       in parameter vector \"sequence_angleStates\" (default = {1,2,3}, i.e.,
       the Cardan angle sequence) around the angles used as potential states.
       For example, the default is to rotate the x-axis of the world frame
       around angles[1], the new y-axis around angles[2] and the new z-axis
       around angles[3], arriving at frame_a.
   </li>
  </ul>
  <p>
  The quaternions have the slight disadvantage that there is a
  non-linear constraint equation between the 4 quaternions.
  Therefore, at least one non-linear equation has to be solved
  during simulation. A tool might, however, analytically solve this
  simple constraint equation. Using the 3 angles as states has the
  disadvantage that there is a singular configuration in which a
  division by zero will occur. If it is possible to determine in advance
  for an application class that this singular configuration is outside
  of the operating region, the 3 angles might be used as potential
  states by setting <strong>useQuaternions</strong> = <strong>false</strong>.
  </p>
  <p>
  In text books about 3-dimensional mechanics often 3 angles and the
  angular velocity are used as states. This is not the case here, since
  3 angles and their derivatives are used as potential states
  (if useQuaternions = false). The reason
  is that for real-time simulation the discretization formula of the
  integrator might be \"inlined\" and solved together with the body equations.
  By appropriate symbolic transformation the performance is
  drastically increased if angles and their
  derivatives are used as states, instead of angles and the angular
  velocity.
  </p>
  <p>
  Whether or not variables of the body are used as states is usually
  automatically selected by the Modelica translator. If parameter
  <strong>enforceStates</strong> is set to <strong>true</strong> in the \"Advanced\" menu,
  then body variables are forced to be used as states according
  to the setting of parameters \"useQuaternions\" and
  \"sequence_angleStates\".
  </p>
  </html>"));
  end BodyVariableMass;

  package Motors
    model M2000R
      parameter Modelica.Units.SI.Time start_delay = 0;
      Modelica.Blocks.Sources.TimeTable thrustCurve(offset = 0, shiftTime = start_delay, startTime = start_delay, table = [0, 0; 0.1, 1500; 0.2, 2250; 2.1, 2400; 4, 1750; 4.6, 100; 4.7, 0; 500, 0], timeScale = 1) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Motors.Internal.SolidMotorAndTank m2000r(Dext = 0.098, Din_0 = 0.03, Isp = 176, h = 0.732, m_0 = 5.368) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
    equation
      connect(m2000r.frame_b, frame_b) annotation(
        Line(points = {{0, 10}, {0, 98}}));
      connect(thrustCurve.y, m2000r.thrust) annotation(
        Line(points = {{-59, 0}, {-10, 0}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Text(origin = {2, -184}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Rectangle(origin = {0.51, 47.73}, fillColor = {85, 85, 85}, fillPattern = FillPattern.VerticalCylinder, extent = {{-59.94, 48.27}, {59.94, -48.27}}), Polygon(origin = {1, -30}, fillColor = {43, 88, 85}, fillPattern = FillPattern.VerticalCylinder, points = {{-21, 30}, {-51, 10}, {-71, -30}, {71, -30}, {49, 10}, {19, 30}, {5, 30}, {-21, 30}}), Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "M2000R")}));
    end M2000R;

    package Internal
      model GenericMotor
        parameter SI.Time Isp = 180 "Specific impulse in seconds";
        Modelica.Blocks.Sources.Constant const(k = 0) annotation(
          Placement(visible = true, transformation(origin = {-34, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput thrust annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-94, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = 1 / (Modelica.Constants.g_n * Isp)) annotation(
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Forces.WorldForce force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
          Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealOutput mdot annotation(
          Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(thrust, gain.u) annotation(
          Line(points = {{-100, 0}, {24, 0}}, color = {0, 0, 127}));
        connect(const.y, force.force[3]) annotation(
          Line(points = {{-23, -54}, {0, -54}, {0, 38}}, color = {0, 0, 127}));
        connect(const.y, force.force[2]) annotation(
          Line(points = {{-23, -54}, {0, -54}, {0, 38}}, color = {0, 0, 127}));
        connect(thrust, force.force[1]) annotation(
          Line(points = {{-100, 0}, {0, 0}, {0, 38}}, color = {0, 0, 127}));
        connect(force.frame_b, frame_b) annotation(
          Line(points = {{0, 60}, {0, 100}}, color = {95, 95, 95}));
        connect(gain.y, mdot) annotation(
          Line(points = {{48, 0}, {106, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Polygon(origin = {0, 12}, fillColor = {107, 107, 107}, fillPattern = FillPattern.VerticalCylinder, points = {{-6, 78}, {-60, 62}, {-90, 2}, {-100, -78}, {100, -78}, {90, 2}, {60, 62}, {6, 78}, {-6, 78}}), Text(origin = {-10, -172}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
      end GenericMotor;

      package Inertia
        partial model PropellantInertiaBase
          parameter Modelica.Units.SI.Mass m_0 "Initial mass";
          parameter Modelica.Units.SI.Mass m_small = 1e-3 "Smallest mass before setting mdot to 0";
          Interfaces.MassPropertiesOutput massOut annotation(
            Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput mdot annotation(
            Placement(visible = true, transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-88, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        initial equation
          massOut.m = m_0;
        equation
          if massOut.m > m_small then
            der(massOut.m) = -mdot;
          else
            der(massOut.m) = 0;
          end if;
          annotation(
            Diagram,
            Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
        end PropellantInertiaBase;

        model SolidPropellantInertia
          extends RocketControl.Components.Motors.Internal.Inertia.PropellantInertiaBase;
          import Modelica.Constants.pi;
          parameter Modelica.Units.SI.Length Dext "External diameter";
          parameter Modelica.Units.SI.Length Din_0 "Initial internal diameter";
          parameter Modelica.Units.SI.Length h "Height";
          final parameter SI.Volume V_0 = pi * ((Dext / 2) ^ 2 - (Din_0 / 2) ^ 2) * h;
          final parameter SI.Density rho = m_0 / V_0;
          Modelica.Units.SI.Length Din "Internal diameter";
          SI.Volume V "Propellant volume";
        equation
          V = massOut.m / rho;
          Din = sqrt(Dext ^ 2 - 4 * V / pi);
          massOut.I[1, 1] = 1 / 2 * massOut.m * ((Dext / 2) ^ 2 + (Din / 2) ^ 2);
          massOut.I[2, 2] = 1 / 12 * massOut.m * (3 * ((Dext / 2) ^ 2 + (Din / 2) ^ 2) + h ^ 2);
          massOut.I[3, 3] = 1 / 12 * massOut.m * (3 * ((Dext / 2) ^ 2 + (Din / 2) ^ 2) + h ^ 2);
          massOut.I[1, 2] = 0;
          massOut.I[1, 3] = 0;
          massOut.I[2, 1] = 0;
          massOut.I[2, 3] = 0;
          massOut.I[3, 1] = 0;
          massOut.I[3, 2] = 0;
          annotation(
            Icon(graphics = {Rectangle(origin = {0, -13}, fillColor = {113, 88, 25}, fillPattern = FillPattern.VerticalCylinder, extent = {{-60, 85}, {60, -85}})}));
        end SolidPropellantInertia;
      end Inertia;

      model SolidMotorAndTank
        parameter SI.Time Isp = 180 "Specific impulse in seconds";
        parameter Modelica.Units.SI.Length Dext "External diameter";
        parameter Modelica.Units.SI.Length Din_0 "Initial internal diameter";
        parameter Modelica.Units.SI.Length h "Height";
        parameter Modelica.Units.SI.Mass m_0 "Initial mass";
        RocketControl.Components.Motors.Internal.Inertia.SolidPropellantInertia solidPropellantInertia(Dext = Dext, Din_0 = Din_0, h = h, m_0 = m_0) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.BodyVariableMass bodyVariableMass( r_CM = {h / 2, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
          Placement(visible = true, transformation(origin = {40, 4}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealInput thrust annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        RocketControl.Components.Motors.Internal.GenericMotor genericMotor(Isp = Isp) annotation(
          Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(solidPropellantInertia.massOut, bodyVariableMass.massInput) annotation(
          Line(points = {{10, 0}, {34, 0}}));
        connect(bodyVariableMass.frame_a, frame_b) annotation(
          Line(points = {{40, 14}, {40, 60}, {0, 60}, {0, 98}}));
        connect(solidPropellantInertia.mdot, genericMotor.mdot) annotation(
          Line(points = {{-8, 0}, {-30, 0}}, color = {0, 0, 127}));
        connect(genericMotor.thrust, thrust) annotation(
          Line(points = {{-50, 0}, {-100, 0}}, color = {0, 0, 127}));
        connect(genericMotor.frame_b, frame_b) annotation(
          Line(points = {{-40, 10}, {-40, 60}, {0, 60}, {0, 98}}, color = {95, 95, 95}));
        annotation(
          Icon(graphics = {Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "M2000R"), Text(origin = {2, -176}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Polygon(origin = {1, -30}, fillColor = {43, 88, 85}, fillPattern = FillPattern.VerticalCylinder, points = {{-21, 30}, {-51, 10}, {-71, -30}, {71, -30}, {49, 10}, {19, 30}, {5, 30}, {-21, 30}}), Rectangle(origin = {0.51, 47.73}, fillColor = {85, 85, 85}, fillPattern = FillPattern.VerticalCylinder, extent = {{-59.94, 48.27}, {59.94, -48.27}})}));
      end SolidMotorAndTank;
    end Internal;

    model Sustainer
      parameter Modelica.Units.SI.Time start_delay = 0;
      Modelica.Blocks.Sources.TimeTable thrustCurve(offset = 0, shiftTime = start_delay, startTime = start_delay, table = [0, 0; 0.1, 500; 0.2, 1000; 2, 1000; 2.2, 402; 20, 400; 20.2, 0; 500, 0], timeScale = 1) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Motors.Internal.SolidMotorAndTank m2000r(Dext = 0.098, Din_0 = 0.03, Isp = 176, h = 0.732, m_0 = 5.368) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
    equation
      connect(m2000r.frame_b, frame_b) annotation(
        Line(points = {{0, 10}, {0, 98}}));
      connect(thrustCurve.y, m2000r.thrust) annotation(
        Line(points = {{-59, 0}, {-10, 0}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Text(origin = {2, -184}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Rectangle(origin = {0.51, 47.73}, fillColor = {85, 85, 85}, fillPattern = FillPattern.VerticalCylinder, extent = {{-59.94, 48.27}, {59.94, -48.27}}), Polygon(origin = {1, -30}, fillColor = {43, 88, 85}, fillPattern = FillPattern.VerticalCylinder, points = {{-21, 30}, {-51, 10}, {-71, -30}, {71, -30}, {49, 10}, {19, 30}, {5, 30}, {-21, 30}}), Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "SUS")}));
    end Sustainer;
  end Motors;

  package Forces
    model LinearSpring "Spring that acts only on a specific direction and can be enabled / disabled"
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
      extends Interfaces.PartialConditionalEnablePort;
      import Modelica.Mechanics.MultiBody.Frames.resolve2;
      import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
      parameter Real n[3] = {1, 0, 0} "Direction along which the spring is acting";
      parameter SI.Position s_0 = 0 "Undeformed spring length";
      parameter Modelica.Units.SI.ModulusOfElasticity c;
      SI.Position s(fixed = false, start = 0);
      SI.Force f[3];
    equation
      s = resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0) * n;
      frame_b.t = zeros(3);
      frame_a.t = zeros(3);
      if enable then
        f = -c * (s - s_0) * n;
      else
        f = zeros(3);
      end if;
      frame_a.f = f;
      frame_b.f = resolveRelative(-f, frame_a.R, frame_b.R);
      annotation(
        Icon(graphics = {Line(points = {{-100, 0}, {-58, 0}, {-43, -30}, {-13, 30}, {17, -30}, {47, 30}, {62, 0}, {100, 0}}), Line(origin = {7.54808, 60}, points = {{-68, 0}, {40, 0}, {52, 0}}, thickness = 0.75), Line(origin = {49.9866, 59.6635}, points = {{-9.64645, 20}, {10.3536, -7.10543e-15}, {-9.64645, -20}}, thickness = 0.75), Text(origin = {0, 22}, extent = {{-150, -75}, {150, -45}}, textString = "c=%c"), Text(origin = {0, -12}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Text(origin = {4, -176}, lineColor = {0, 0, 255}, extent = {{-150, 85}, {150, 45}}, textString = "%name"), Line(origin = {-49.65, 60}, points = {{9.64645, 20}, {-10.3536, -7.10543e-15}, {9.64645, -20}}, thickness = 0.75)}));
    end LinearSpring;

    model LinearDamper "Damper that acts only on a specific direction and can be enabled / disabled"
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
      extends Interfaces.PartialConditionalEnablePort;
      import Modelica.Mechanics.MultiBody.Frames.resolve2;
      import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
      parameter Real n[3] = {1, 0, 0} "Direction along which the damper is acting";
      parameter Modelica.Units.SI.DampingCoefficient d;
      SI.Position s(fixed = false, start = 0);
      SI.Force f[3];
    equation
      s = resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0) * n;
      frame_b.t = zeros(3);
      frame_a.t = zeros(3);
      if enable then
        f = -d * der(s) * n;
      else
        f = zeros(3);
      end if;
      frame_a.f = f;
      frame_b.f = resolveRelative(-f, frame_a.R, frame_b.R);
      annotation(
        Icon(graphics = {Text(origin = {4, -176}, lineColor = {0, 0, 255}, extent = {{-150, 85}, {150, 45}}, textString = "%name"), Text(origin = {0, -12}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Rectangle(origin = {8, 0}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {30, -30}}), Line(origin = {7.40383, -1.76063e-06}, points = {{-101, 0}, {-60, 0}}), Line(origin = {7.40383, -1.76063e-06}, points = {{30, 0}, {100, 0}}), Line(origin = {7.40383, -1.76063e-06}, points = {{-60, -30}, {-60, 30}}), Text(origin = {0, 18}, extent = {{-150, -75}, {150, -45}}, textString = "d=%d"), Line(visible = false, origin = {7.40383, -1.76063e-06}, points = {{-100, -99}, {-100, -25}, {-10, -25}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Line(origin = {7.40383, -1.76063e-06}, points = {{-60, -30}, {60, -30}}), Line(origin = {7.40383, -1.76063e-06}, points = {{-60, 30}, {60, 30}}), Line(origin = {49.9866, 59.6635}, points = {{-9.64645, 20}, {10.3536, -7.10543e-15}, {-9.64645, -20}}, thickness = 0.75), Line(origin = {-49.65, 60}, points = {{9.64645, 20}, {-10.3536, -7.10543e-15}, {9.64645, -20}}, thickness = 0.75), Line(origin = {7.54808, 60}, points = {{-68, 0}, {40, 0}, {52, 0}}, thickness = 0.75)}));
    end LinearDamper;

    model LinearSpringDamperParallel "Spring and damper in parallel that act only on a specific direction and can be enabled / disabled"
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
      extends Interfaces.PartialConditionalEnablePort;
      parameter Real n[3] = {1, 0, 0} "Direction along which the spring is acting";
      parameter SI.Length s_0 = 0 "Undeformed spring length";
      parameter SI.Length s_start = 0 "Inital deformation";
      parameter Boolean s_fixed = false;
      parameter SI.ModulusOfElasticity c;
      parameter SI.DampingCoefficient d;
      Components.Forces.LinearSpring linearSpring(c = c, n = n, s(fixed = s_fixed, start = s_start), s_0 = s_0, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.Forces.LinearDamper linearDamper(d = d, n = n, s(start = s_start), useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(enable, linearDamper.enable) annotation(
        Line(points = {{0, 106}, {0, 80}, {40, 80}, {40, -20}, {0, -20}, {0, -30}}, color = {255, 0, 255}));
      connect(enable, linearSpring.enable) annotation(
        Line(points = {{0, 106}, {0, 50}}, color = {255, 0, 255}));
      connect(linearSpring.frame_b, frame_b) annotation(
        Line(points = {{10, 40}, {60, 40}, {60, 0}, {100, 0}}));
      connect(linearDamper.frame_b, frame_b) annotation(
        Line(points = {{10, -40}, {60, -40}, {60, 0}, {100, 0}}, color = {95, 95, 95}));
      connect(frame_a, linearDamper.frame_a) annotation(
        Line(points = {{-100, 0}, {-60, 0}, {-60, -40}, {-10, -40}}));
      connect(linearSpring.frame_a, frame_a) annotation(
        Line(points = {{-10, 40}, {-60, 40}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
      annotation(
        Icon(graphics = {Line(points = {{-52, -40}, {68, -40}}), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}, endAngle = 360), Text(origin = {-82, 122}, extent = {{-150, -75}, {150, -45}}, textString = "d=%d"), Line(points = {{38, -70}, {80, -70}}), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}, endAngle = 360), Line(points = {{80, 40}, {80, -70}}), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Text(origin = {-2, 52}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Text(visible = false, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Text(visible = false, origin = {0, -28}, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Rectangle(fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-52, -40}, {38, -100}}), Line(points = {{-80, 40}, {-60, 40}, {-45, 10}, {-15, 70}, {15, 10}, {45, 70}, {60, 40}, {80, 40}}), Text(origin = {-82, 154}, extent = {{-150, -75}, {150, -45}}, textString = "c=%c"), Line(points = {{-80, -70}, {-52, -70}}), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}, endAngle = 360), Line(points = {{80, 0}, {100, 0}}), Line(points = {{-52, -100}, {68, -100}}), Line(visible = false, points = {{-100, -101}, {-100, -80}, {-6, -80}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}, endAngle = 360), Line(origin = {-80, -15}, points = {{0, 55}, {0, -55}}), Line(origin = {-89, 0}, points = {{9, 0}, {-7, 0}, {-9, 0}}), Line(origin = {18.3173, -69.7805}, points = {{-68, 0}, {16, 0}, {16, 0}}, thickness = 0.75), Line(origin = {-39.4628, -69.8086}, points = {{9.64645, 20}, {-10.3536, -7.10543e-15}, {9.64645, -20}}, thickness = 0.75), Line(origin = {24.2344, -69.8295}, points = {{-9.64645, 20}, {10.3536, -7.10543e-15}, {-9.64645, -20}}, thickness = 0.75)}));
    end LinearSpringDamperParallel;

    model ConnectionFlange
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
    
    parameter SI.Length diameter "Flange diameter";
    parameter SI.TranslationalSpringConstant c(final min=0) "Spring constant";
    parameter SI.Length s_unstretched=0 "Unstretched spring length";
    parameter SI.TranslationalDampingConstant d(final min=0) = 0
      "Damping constant";
        
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0.5, sqrt(3) / 2} * r) annotation(
        Placement(visible = true, transformation(origin = {-50, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r = {0, 0, -1} * r) annotation(
        Placement(visible = true, transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel1(c = c, d = d, s_unstretched = s_unstretched) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel2(c = c, d = d, s_unstretched = s_unstretched) annotation(
        Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r = {0, -0.5, -sqrt(3) / 2} * r) annotation(
        Placement(visible = true, transformation(origin = {50, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r = {0, 0.5, -sqrt(3) / 2} * r) annotation(
        Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r = {0, 0, 1} * r) annotation(
        Placement(visible = true, transformation(origin = {50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
    final parameter SI.Length r = diameter / 2;
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {0, -0.5, sqrt(3) / 2} * r)  annotation(
        Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = c, d = d, s_unstretched = s_unstretched)  annotation(
        Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(fixedTranslation.frame_a, frame_a) annotation(
        Line(points = {{-60, 60}, {-80, 60}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation1.frame_a, frame_a) annotation(
        Line(points = {{-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_a, frame_a) annotation(
        Line(points = {{-60, -60}, {-80, -60}, {-80, 0}, {-100, 0}}));
  connect(fixedTranslation.frame_b, springDamperParallel.frame_a) annotation(
        Line(points = {{-40, 60}, {-10, 60}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_b, springDamperParallel2.frame_a) annotation(
        Line(points = {{-40, -60}, {-10, -60}}));
  connect(fixedTranslation1.frame_b, springDamperParallel1.frame_a) annotation(
        Line(points = {{-40, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(springDamperParallel2.frame_b, fixedTranslation5.frame_a) annotation(
        Line(points = {{10, -60}, {40, -60}}, color = {95, 95, 95}));
  connect(fixedTranslation5.frame_b, frame_b) annotation(
        Line(points = {{60, -60}, {80, -60}, {80, 0}, {100, 0}}));
  connect(springDamperParallel1.frame_b, fixedTranslation4.frame_a) annotation(
        Line(points = {{10, 0}, {40, 0}}));
  connect(fixedTranslation4.frame_b, frame_b) annotation(
        Line(points = {{60, 0}, {100, 0}}));
  connect(fixedTranslation3.frame_b, frame_b) annotation(
        Line(points = {{60, 60}, {80, 60}, {80, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(springDamperParallel.frame_b, fixedTranslation3.frame_a) annotation(
        Line(points = {{10, 60}, {40, 60}}));
      annotation(
        Icon(graphics = {Rectangle(origin = {-20, 0}, fillColor = {107, 107, 107}, fillPattern = FillPattern.Solid, extent = {{-20, 60}, {20, -60}}), Rectangle(origin = {20, 0}, fillColor = {147, 147, 147}, fillPattern = FillPattern.Solid, extent = {{20, 60}, {-20, -60}}), Line(origin = {-79, 0}, points = {{39, 0}, {-19, 0}}), Line(origin = {81, 0}, points = {{-41, 0}, {21, 0}}), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Text(extent = {{-150, 105}, {150, 135}}, textString = "c=%c"), Text(extent = {{-150, 70}, {150, 100}}, textString = "d=%d")}));
    end ConnectionFlange;
  end Forces;

  package LaunchPad
    model LaunchLug
      extends Interfaces.PartialConditionalEnablePort;
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
      parameter Modelica.Units.SI.ModulusOfElasticity c_ax = 1 "Axial spring stiffness";
      parameter Modelica.Units.SI.DampingCoefficient d_ax = 1 "Axial dampening coefficient";
      parameter Modelica.Units.SI.ModulusOfElasticity c_norm = 1 "Normal spring stiffness";
      parameter Modelica.Units.SI.DampingCoefficient d_norm = 1 "Normal dampening coefficient";
      parameter SI.Length lug_length;
      parameter Real n_ax[3] = {0, 0, 1} "Axial direction of the launch lug (constrained movement)";
      parameter Real n_free[3] = {1, 0, 0} "Direction where the relative movement of frame_a and frame_b is not constrained (with respect to frame_a)";
      parameter SI.Length s_0_ax = 0 "Undeformed axial spring length";
      parameter SI.Length s_start_ax = 0 "Inital axial spring deformation";
      parameter Boolean s_fixed_ax = false;
      parameter SI.Length s_0_norm = 0 "Undeformed axial spring length";
      parameter SI.Length s_start_norm = 0 "Inital axial spring deformation";
      parameter Boolean s_fixed_norm = false;
      parameter Boolean s_fixed_norm2 = false;
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation lugTranslation1(r = -n_ax * lug_length) annotation(
        Placement(visible = true, transformation(origin = {20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Components.Forces.LinearSpringDamperParallel normalRailSpring1(c = c_norm / 2, d = d_norm / 2, n = n_norm, s_0 = s_0_norm, s_start = s_start_norm, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Components.Forces.LinearSpringDamperParallel normalRailSpring2(c = c_norm / 2, d = d_norm / 2, n = n_norm, s_0 = s_0_norm, s_fixed = s_fixed_norm2, s_start = s_start_norm, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Components.Forces.LinearSpringDamperParallel axialRailSpring(c = c_ax, d = d_ax, n = n_ax, s_0 = s_0_ax, s_start = s_start_ax, s_fixed = s_fixed_ax, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation lugTranslation2(r = n_ax * lug_length) annotation(
        Placement(visible = true, transformation(origin = {20, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
      parameter Real n_norm[3] = cross(n_ax, n_free) "Normal direction of the launch lug";
    equation
      connect(frame_a, axialRailSpring.frame_a) annotation(
        Line(points = {{-100, 0}, {-80, 0}, {-80, 40}, {-60, 40}}));
      connect(normalRailSpring1.frame_b, lugTranslation1.frame_a) annotation(
        Line(points = {{50, 10}, {50, 40}, {30, 40}}, color = {95, 95, 95}));
      connect(lugTranslation1.frame_b, normalRailSpring2.frame_b) annotation(
        Line(points = {{10, 40}, {-10, 40}, {-10, 10}}, color = {95, 95, 95}));
      connect(axialRailSpring.frame_b, lugTranslation1.frame_b) annotation(
        Line(points = {{-40, 40}, {10, 40}}));
      connect(frame_a, lugTranslation2.frame_a) annotation(
        Line(points = {{-100, 0}, {-80, 0}, {-80, -40}, {10, -40}}));
      connect(lugTranslation2.frame_b, normalRailSpring1.frame_a) annotation(
        Line(points = {{30, -40}, {50, -40}, {50, -10}}, color = {95, 95, 95}));
      connect(lugTranslation2.frame_a, normalRailSpring2.frame_a) annotation(
        Line(points = {{10, -40}, {-10, -40}, {-10, -10}}));
      connect(normalRailSpring2.enable, enable) annotation(
        Line(points = {{-20, 0}, {-28, 0}, {-28, 60}, {0, 60}, {0, 106}}, color = {255, 0, 255}));
      connect(axialRailSpring.enable, enable) annotation(
        Line(points = {{-50, 50}, {-50, 60}, {0, 60}, {0, 106}}, color = {255, 0, 255}));
      connect(normalRailSpring1.enable, enable) annotation(
        Line(points = {{40, 0}, {20, 0}, {20, 20}, {-28, 20}, {-28, 60}, {0, 60}, {0, 106}}, color = {255, 0, 255}));
      connect(lugTranslation1.frame_a, frame_b) annotation(
        Line(points = {{30, 40}, {80, 40}, {80, 0}, {100, 0}}, color = {95, 95, 95}));
      annotation(
        Icon(graphics = {Polygon(origin = {-7, 2}, fillColor = {139, 139, 139}, fillPattern = FillPattern.HorizontalCylinder, points = {{101, 28}, {101, -28}, {-53, -28}, {-53, -74}, {-89, -24}, {-89, 18}, {-53, 68}, {-53, 28}, {9, 28}, {101, 28}}), Text(lineColor = {0, 0, 255}, extent = {{-150, 85}, {150, 45}}, textString = "%name")}));
    end LaunchLug;

    model LaunchRail
      parameter Modelica.Units.NonSI.Angle_deg azimuth;
      parameter Modelica.Units.NonSI.Angle_deg elevation;
      parameter SI.Distance lug_length;
      parameter SI.Distance rail_length;
      parameter SI.ModulusOfElasticity c_x;
      parameter SI.DampingCoefficient d_x;
      parameter SI.ModulusOfElasticity c_y;
      parameter SI.DampingCoefficient d_y;
      parameter SI.ModulusOfElasticity c_z;
      parameter SI.DampingCoefficient d_z;
      parameter SI.Position r_rel[3] = {0, 0, 0} "Relative position between the ramp base and the aft flange";
      Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(angle_1(fixed = true), angle_2(fixed = true), angle_3(fixed = true), angle_d_1(fixed = false), angle_d_2(fixed = false), angle_d_3(fixed = false), r_rel_a_1(fixed = true, start = r_rel[1]), r_rel_a_2(fixed = true, start = r_rel[2]), r_rel_a_3(fixed = true, start = r_rel[3]), sequence_start = {3, 2, 1}, use_angle = true, use_r = true, use_v = true, use_w = true, v_rel_a_1(fixed = true), v_rel_a_2(fixed = true), v_rel_a_3(fixed = true), w_rel_b_1(fixed = true), w_rel_b_2(fixed = true), w_rel_b_3(fixed = true)) annotation(
        Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      LaunchLug lug_aft(c_ax = c_z / 2, c_norm = c_y / 2, d_ax = d_z / 2, d_norm = d_y / 2, lug_length = lug_length, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      LaunchLug lug_bow(c_ax = c_z / 2, c_norm = c_y / 2, d_ax = d_z / 2, d_norm = d_y / 2, lug_length = lug_length, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {-10, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Forces.LinearSpringDamperParallel linearSpringDamperParallel(c = c_x, d = d_x, n = {1, 0, 0}, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Mechanics.MultiBody.Parts.FixedRotation launchRailAngle(angles = {azimuth, elevation, 0}, rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence, sequence = {3, 2, 1}) annotation(
        Placement(visible = true, transformation(origin = {-62, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
        Placement(visible = true, transformation(origin = {-100, -50}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b_lug_bow annotation(
        Placement(visible = true, transformation(origin = {100, 80}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b_lug_aft annotation(
        Placement(visible = true, transformation(origin = {100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant annotation(
        Placement(visible = true, transformation(origin = {-108, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(frame_a, launchRailAngle.frame_a) annotation(
        Line(points = {{-100, -50}, {-72, -50}}));
      connect(freeMotionScalarInit.frame_a, launchRailAngle.frame_b) annotation(
        Line(points = {{20, -10}, {-40, -10}, {-40, -50}, {-52, -50}}, color = {95, 95, 95}));
      connect(launchRailAngle.frame_b, linearSpringDamperParallel.frame_a) annotation(
        Line(points = {{-52, -50}, {-40, -50}, {-40, -90}, {70, -90}, {70, -80}}, color = {95, 95, 95}));
      connect(linearSpringDamperParallel.frame_b, frame_b_lug_aft) annotation(
        Line(points = {{70, -60}, {70, 20}, {100, 20}}));
      connect(freeMotionScalarInit.frame_b, frame_b_lug_aft) annotation(
        Line(points = {{40, -10}, {60, -10}, {60, 20}, {100, 20}}));
      connect(lug_bow.frame_a, launchRailAngle.frame_b) annotation(
        Line(points = {{-20, 68}, {-40, 68}, {-40, -50}, {-52, -50}}, color = {95, 95, 95}));
      connect(lug_bow.frame_b, frame_b_lug_bow) annotation(
        Line(points = {{0, 68}, {50, 68}, {50, 80}, {100, 80}}));
      connect(lug_aft.frame_b, frame_b_lug_aft) annotation(
        Line(points = {{0, 20}, {100, 20}}, color = {95, 95, 95}));
      connect(lug_aft.frame_a, launchRailAngle.frame_b) annotation(
        Line(points = {{-20, 20}, {-40, 20}, {-40, -50}, {-52, -50}}));
  connect(booleanConstant.y, lug_bow.enable) annotation(
        Line(points = {{-96, 50}, {-10, 50}, {-10, 78}}, color = {255, 0, 255}));
  connect(booleanConstant.y, lug_aft.enable) annotation(
        Line(points = {{-96, 50}, {-10, 50}, {-10, 30}}, color = {255, 0, 255}));
  connect(booleanConstant.y, linearSpringDamperParallel.enable) annotation(
        Line(points = {{-96, 50}, {60, 50}, {60, -70}}, color = {255, 0, 255}));
      annotation(
        Icon(graphics = {Polygon(origin = {-20, -6}, fillColor = {85, 122, 162}, fillPattern = FillPattern.VerticalCylinder, points = {{-28, -42}, {-14, -50}, {-14, -42}, {26, 38}, {28, 50}, {20, 42}, {-20, -38}, {-28, -42}}), Polygon(origin = {-9, 8}, fillColor = {222, 222, 222}, fillPattern = FillPattern.Forward, points = {{17, 88}, {-71, -78}, {-71, -88}, {71, -88}, {49, -80}, {-55, -80}, {17, 62}, {17, 88}}), Rectangle(origin = {6, 20}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Rectangle(origin = {-22, -36}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Text(lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name"), Text(origin = {-760.66, 76}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "a"), Text(origin = {-566.66, 74}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "bow"), Text(origin = {-227.774, 14}, lineColor = {128, 128, 128}, extent = {{277.774, -29}, {377.774, -58}}, textString = "aft"), Line(origin = {40, -48}, points = {{-62, 12}, {20, 12}, {20, -12}, {62, -12}}), Line(origin = {53, 40}, points = {{-47, -20}, {7, -20}, {7, 20}, {47, 20}})}));
    end LaunchRail;

    model SurfaceFixed
    parameter NonSI.Angle_deg latitude;
    parameter NonSI.Angle_deg longitude;
    equation

      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end SurfaceFixed;
  end LaunchPad;

  package Control
    model PitchController
      RocketControl.Interfaces.FinDeflectionOutput finDeflectionOutput[4] annotation(
        Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Interfaces.Debug.RealToDeflection realToDeflection annotation(
        Placement(visible = true, transformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Interfaces.Debug.RealToDeflection realToDeflection1 annotation(
        Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Interfaces.Debug.RealToDeflection realToDeflection2 annotation(
        Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Interfaces.Debug.RealToDeflection realToDeflection3 annotation(
        Placement(visible = true, transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 0) annotation(
        Placement(visible = true, transformation(origin = {-84, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain gain(k = -1) annotation(
        Placement(visible = true, transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(realToDeflection.finDeflectionOutput, finDeflectionOutput[1]) annotation(
        Line(points = {{10, 70}, {60, 70}, {60, 0}, {98, 0}}));
      connect(realToDeflection1.finDeflectionOutput, finDeflectionOutput[2]) annotation(
        Line(points = {{10, 30}, {40, 30}, {40, 0}, {98, 0}}));
      connect(realToDeflection2.finDeflectionOutput, finDeflectionOutput[3]) annotation(
        Line(points = {{10, -30}, {40, -30}, {40, 0}, {98, 0}}));
      connect(realToDeflection3.finDeflectionOutput, finDeflectionOutput[4]) annotation(
        Line(points = {{10, -70}, {60, -70}, {60, 0}, {98, 0}}));
      connect(const.y, realToDeflection2.u) annotation(
        Line(points = {{-72, 84}, {-58, 84}, {-58, -30}, {-10, -30}}, color = {0, 0, 127}));
      connect(const.y, realToDeflection.u) annotation(
        Line(points = {{-72, 84}, {-58, 84}, {-58, 70}, {-10, 70}}, color = {0, 0, 127}));
      connect(u, realToDeflection1.u) annotation(
        Line(points = {{-100, 0}, {-50, 0}, {-50, 30}, {-10, 30}}, color = {0, 0, 127}));
      connect(gain.y, realToDeflection3.u) annotation(
        Line(points = {{-18, -70}, {-10, -70}}, color = {0, 0, 127}));
      connect(u, gain.u) annotation(
        Line(points = {{-100, 0}, {-62, 0}, {-62, -70}, {-42, -70}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Text(extent = {{-100, 80}, {100, -80}}, textString = "Pitch")}));
    end PitchController;
  end Control;

  package Noise
    model NormalNoiseAndRandomWalk
      import RocketControl.Types.*;
      extends Modelica.Blocks.Interfaces.SO;
      parameter SI.Duration samplePeriod "Noise sample period";
      parameter Real sigmaNoise = 1 "Standard Deviation of the white noise";
      parameter Real sigmaRW = 1 "Standard Deviation of the white noise generating the Random Walk";
      Modelica.Blocks.Math.Add add annotation(
        Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.Integrator integrator annotation(
        Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Noise.NormalNoise randomWalk(samplePeriod = samplePeriod, sigma = sigmaRW, useAutomaticLocalSeed = true, useGlobalSeed = true) annotation(
        Placement(visible = true, transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Noise.NormalNoise noise(samplePeriod = samplePeriod, sigma = sigmaNoise, useAutomaticLocalSeed = true, useGlobalSeed = true) annotation(
        Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(add.y, y) annotation(
        Line(points = {{82, 0}, {110, 0}}, color = {0, 0, 127}));
      connect(integrator.y, add.u1) annotation(
        Line(points = {{22, 30}, {32, 30}, {32, 6}, {58, 6}}, color = {0, 0, 127}));
      connect(randomWalk.y, integrator.u) annotation(
        Line(points = {{-38, 30}, {-2, 30}}, color = {0, 0, 127}));
      connect(noise.y, add.u2) annotation(
        Line(points = {{-38, -10}, {32, -10}, {32, -6}, {58, -6}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{94, -14}, {72, -6}, {72, -22}, {94, -14}}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-76, 90}, {-84, 68}, {-68, 68}, {-76, 90}}), Text(visible = false, extent = {{-75, 42}, {95, 2}}, textString = "%y_off"), Line(points = {{-76, 68}, {-76, -80}}, color = {192, 192, 192}), Line(points = {{-86, -14}, {72, -14}}, color = {192, 192, 192}), Text(visible = false, lineColor = {238, 46, 47}, extent = {{-92, 20}, {98, -22}}, textString = "%fixedLocalSeed"), Line(visible = false, points = {{-76, 48}, {72, 48}}), Line(points = {{-76, -19}, {-62, -19}, {-62, -3}, {-54, -3}, {-54, -51}, {-46, -51}, {-46, -29}, {-38, -29}, {-38, 55}, {-30, 55}, {-30, 23}, {-30, 23}, {-30, -37}, {-20, -37}, {-20, -19}, {-10, -19}, {-10, -47}, {0, -47}, {0, 35}, {6, 35}, {6, 49}, {12, 49}, {12, -7}, {22, -7}, {22, 5}, {28, 5}, {28, -25}, {38, -25}, {38, 47}, {48, 47}, {48, 13}, {56, 13}, {56, -53}, {66, -53}}), Text(origin = {0, 80}, lineColor = {175, 175, 175}, extent = {{-100, 20}, {100, -20}}, textString = "PSD=%noisePSD"), Text(origin = {0, -80}, lineColor = {175, 175, 175}, extent = {{-100, 20}, {100, -20}}, textString = "BS=%biasStability"), Text(extent = {{-150, -110}, {150, -150}}, textString = "%samplePeriod s")}));
    end NormalNoiseAndRandomWalk;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Noise;

  package Frames
    model ECEF
    outer RocketControl.World.MyWorld world;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    parameter SI.AngularVelocity w = 72.92115e-6;
    SI.Angle phi;
    
    initial equation
    phi = 0;
//frame_b.R = Modelica.Mechanics.MultiBody.Frames.nullRotation();
    equation
    
    der(phi) = w;
    Connections.root(frame_b.R);
    
    
  frame_b.r_0 = {0,0,0};
    frame_b.R = Modelica.Mechanics.MultiBody.Frames.planarRotation({0,0,1}, phi, w);
//zeros(3) = fixed.frame_b.f ;
//zeros(3) = fixed.frame_b.t;
      annotation(
        Icon(graphics = {Ellipse(lineColor = {0, 170, 255}, fillColor = {65, 188, 96}, fillPattern = FillPattern.Sphere, extent = {{-100, 100}, {100, -100}}, endAngle = 360), Line(origin = {46, 0}, points = {{-46, 0}, {46, 0}}, thickness = 1), Line(origin = {-32.21, 17.79}, points = {{32.2071, 82.2071}, {32.2071, -17.7929}, {-31.7929, -81.7929}}, thickness = 1), Ellipse(extent = {{-100, 100}, {100, -100}}, endAngle = 360), Line(origin = {-61, -61}, points = {{-3, 3}, {-3, -3}, {3, -3}}, thickness = 1), Line(origin = {0, 97.65}, points = {{-4, -1.64645}, {0, 2.35355}, {4, -1.64645}}, thickness = 1), Line(origin = {89.83, 0}, points = {{-1.64645, 4}, {2.35355, 0}, {-1.64645, -4}}, thickness = 1), Text(origin = {-45, -67}, extent = {{-15, 15}, {15, -15}}, textString = "x"), Text(origin = {85, 19}, extent = {{-15, 15}, {15, -15}}, textString = "y"), Text(origin = {19, 93}, extent = {{-15, 15}, {15, -15}}, textString = "z"), Text(origin = {31, -55}, extent = {{-53, 21}, {53, -21}}, textString = "ECEF")}));
    end ECEF;

    model toNED
    outer RocketControl.World.MyWorld world;
    import Modelica.Mechanics.MultiBody.Frames;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    //Frames.Orientation R_rel;
      //Frames.Orientation R_rel_inv;
    Real T[3,3];
    SI.AngularVelocity w[3];
    equation
//R_rel = world.eci2nedOrientation(frame_a.r_0, der(frame_a.r_0));
//R_rel_inv = Modelica.Mechanics.MultiBody.Frames.Orientation(transpose(R_rel.T), zeros(3));
      Connections.branch(frame_a.R, frame_b.R);
      assert(cardinality(frame_a) > 0 or cardinality(frame_b) > 0,
        "Neither connector frame_a nor frame_b of NED object is connected");
//  /* Relationships between quantities of frame_a and frame_b */
      frame_b.r_0 = frame_a.r_0;
  if Connections.rooted(frame_a.R) then
        (T, w) = world.eci2nedTransform(frame_a.r_0, der(frame_a.r_0));
//Frames.absoluteRotation(frame_a.R, R_rel);
        frame_b.R = Modelica.Mechanics.MultiBody.Frames.Orientation(T, w);
//    zeros(3) = frame_a.f + frame_b.f;
//    zeros(3) = frame_a.t + frame_b.f;
        zeros(3) = frame_a.f + Frames.resolveRelative(frame_b.f, frame_b.R, frame_a.R);
        zeros(3) = frame_a.t + Frames.resolveRelative(frame_b.t, frame_b.R, frame_a.R);
      else
        frame_a.R = Modelica.Mechanics.MultiBody.Frames.inverseRotation(Modelica.Mechanics.MultiBody.Frames.Orientation(T, w));
        zeros(3) = frame_b.f + Frames.resolveRelative(frame_a.f, frame_a.R, frame_b.R);
        zeros(3) = frame_b.t + Frames.resolveRelative(frame_a.t, frame_a.R, frame_b.R);
      end if;      /*
      if checkTotalPower then
        totalPower = frame_a.f*Frames.resolve2(frame_a.R, der(frame_a.r_0)) +
                     frame_b.f*Frames.resolve2(frame_b.R, der(frame_b.r_0)) +
                     frame_a.t*Frames.angularVelocity2(frame_a.R) +
                     frame_b.t*Frames.angularVelocity2(frame_b.R);
      else
        totalPower = 0;
      end if;
    */
      annotation(
        Icon(graphics = {Text(extent = {{-90, 90}, {90, -90}}, textString = "ecef2ned")}));
    end toNED;

    model EarthSurfaceTranslation
      outer RocketControl.World.MyWorld world;
      parameter NonSI.Angle_deg latitude;
      parameter NonSI.Angle_deg longitude;
      parameter SI.Position altitude;
      
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = world.WGS84.lla2ecef(latitude, longitude, altitude))  annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(frame_a, fixedTranslation.frame_a) annotation(
        Line(points = {{-100, 0}, {-10, 0}}));
  connect(fixedTranslation.frame_b, frame_b) annotation(
        Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
      annotation(
        Icon(graphics = {Text(lineColor = {128, 128, 128}, extent = {{-89, 38}, {-53, 13}}, textString = "a"), Text(origin = {0, 4}, extent = {{150, -50}, {-150, -20}}, textString = "lat=%latitude"), Text(lineColor = {128, 128, 128}, extent = {{57, 39}, {93, 14}}, textString = "b"), Rectangle(fillPattern = FillPattern.Solid, extent = {{-99, 5}, {101, -5}}), Text(origin = {0, 4}, extent = {{150, -50}, {-150, -20}}, textString = "lat=%latitude"), Text(origin = {0, -54}, extent = {{150, -50}, {-150, -20}}, textString = "alt=%altitude"), Text(origin = {0, -24}, extent = {{150, -50}, {-150, -20}}, textString = "lon=%longitude")}));
    end EarthSurfaceTranslation;

    model eci2ecef
      import Modelica.Mechanics.MultiBody.Frames;
      
      outer RocketControl.World.MyWorld world;
      
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      
      Frames.Orientation R_rel;
      Frames.Orientation R_rel_inv;
      
      Real T[3,3];
      SI.AngularVelocity w[3];
      
    equation
    
      (T, w) = world.eci2ecefTransform(time);
      
      R_rel = Modelica.Mechanics.MultiBody.Frames.Orientation(T, w);
      R_rel_inv = Modelica.Mechanics.MultiBody.Frames.Orientation(transpose(R_rel.T), -w);
      
      Connections.branch(frame_a.R, frame_b.R);
      assert(cardinality(frame_a) > 0 or cardinality(frame_b) > 0,
        "Neither connector frame_a nor frame_b of eci2ecef object is connected");
    
      /* Relationships between quantities of frame_a and frame_b */
      frame_b.r_0 = frame_a.r_0;
      if Connections.rooted(frame_a.R) then
        frame_b.R = Frames.absoluteRotation(frame_a.R, R_rel);
        zeros(3) = frame_a.f + Frames.resolve1(R_rel, frame_b.f);
        zeros(3) = frame_a.t + Frames.resolve1(R_rel, frame_b.t);
      else
        frame_a.R = Frames.absoluteRotation(frame_b.R, R_rel_inv);
        zeros(3) = frame_b.f + Frames.resolve1(R_rel_inv, frame_a.f);
        zeros(3) = frame_b.t + Frames.resolve1(R_rel_inv, frame_a.t);
      end if;
    
      /*
      if checkTotalPower then
        totalPower = frame_a.f*Frames.resolve2(frame_a.R, der(frame_a.r_0)) +
                     frame_b.f*Frames.resolve2(frame_b.R, der(frame_b.r_0)) +
                     frame_a.t*Frames.angularVelocity2(frame_a.R) +
                     frame_b.t*Frames.angularVelocity2(frame_b.R);
      else
        totalPower = 0;
      end if;
    */
      annotation(
        Icon(graphics = {Text(extent = {{-90, 90}, {90, -90}}, textString = "eci2ecef")}));
    end eci2ecef;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Frames;

  model RelativePositionInit
    "Initial conditions given on relative position resolved in frame_a"
    parameter SI.Position r_rel_b_start[3]= {0, 0, 0}
      "Initial position of frame_b resolved in frame_a";
    SI.Position r_rel_b[3];
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
      annotation (Placement(transformation(extent={{-120,-20},{-80,20}},
            rotation=0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b
      annotation (Placement(transformation(extent={{80,-20},{120,20}},
            rotation=0)));
  equation
    frame_a.f = zeros(3);
    frame_b.f = zeros(3);
    frame_a.t = zeros(3);
    frame_b.t = zeros(3);
    r_rel_b = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_b.R, frame_b.r_0 - frame_a.r_0);
  initial equation
    r_rel_b = r_rel_b_start;
    der(r_rel_b) = zeros(3);
    annotation (Diagram(graphics),
                         Icon(graphics={
          Line(points={{-80,0},{80,0}}, color={0,0,0}),
          Line(points={{-46,20},{-80,0},{-48,-20}}, color={0,0,0}),
          Line(points={{46,20},{80,0},{48,-20}}, color={0,0,0})}));
  end RelativePositionInit;
end Components;
