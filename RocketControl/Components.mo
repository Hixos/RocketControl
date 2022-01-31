within RocketControl;

package Components
  package Sensors
    package IdealSensors
      model IdealGyroscope
        extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
        
        outer World world;
        Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput w[3](each final unit="rad/s", each final quantity="AngularVelocity",each displayUnit="deg/s") annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
          Line(points = {{-100, 0}, {-10, 0}}));
  connect(absoluteAngularVelocity.w, w) annotation(
          Line(points = {{12, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "rad/s")}));
      end IdealGyroscope;

      model IdealAccelerometer
        extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
        outer World world;
        SI.Acceleration acc_inertial[3];
        SI.Acceleration acc_body[3];
        Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
          Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          
          Modelica.Blocks.Interfaces.RealOutput a[3](each final unit="m/s2", each final quantity="Acceleration") annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        acc_inertial = der(absoluteVelocity.v);
        acc_body = acc_inertial - world.gravityAcceleration(frame_a.r_0);
        a = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_a.R, acc_body);
        connect(frame_a, absoluteVelocity.frame_a) annotation(
          Line(points = {{-100, 0}, {-70, 0}}));
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "m/s^2")}));
      end IdealAccelerometer;

      model IdealMagnetometer
         extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
        //  outer World.Interfaces.WorldBase world;
        outer World.MyWorld world;
        
        Modelica.Blocks.Interfaces.RealOutput b[3](each final unit="T", each final quantity="MagneticFluxDensity",each displayUnit="nT") annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        b = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_a.R, world.magneticField(frame_a.r_0));
        assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
        frame_a.f = zeros(3);
        frame_a.t = zeros(3);
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "T")}));
      end IdealMagnetometer;

      model IdealBarometer
        extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
        outer RocketControl.World.Atmosphere atmosphere;
        outer RocketControl.World.MyWorld world;
        
        Modelica.Blocks.Interfaces.RealOutput p(final unit="Pa", final quantity="Pressure", displayUnit="kPa") annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        p = atmosphere.pressure(world.altitude(frame_a.r_0));
        assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
        frame_a.f = zeros(3);
        frame_a.t = zeros(3);
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "Pa"), Line(origin = {85.1063, -1.10634}, points = {{-15.1063, 1.10634}, {6.89366, 1.10634}, {14.8937, -0.893661}})}));
      end IdealBarometer;

      model IdealGNSS
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
        Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition position(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
          Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput x[3](each final unit = "m", each final quantity = "Position") annotation(
          Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput v[3](each final unit = "m/s", each final quantity = "Velocity") annotation(
          Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity velocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
          Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(frame_a, position.frame_a) annotation(
          Line(points = {{-100, 0}, {-55, 0}, {-55, 40}, {-10, 40}}));
        connect(position.r, x) annotation(
          Line(points = {{11, 40}, {110, 40}}, color = {0, 0, 127}, thickness = 0.5));
  connect(frame_a, velocity.frame_a) annotation(
          Line(points = {{-100, 0}, {-54, 0}, {-54, -40}, {-10, -40}}));
  connect(velocity.v, v) annotation(
          Line(points = {{12, -40}, {110, -40}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "GNSS"), Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Line(origin = {90, 0}, points = {{10, 40}, {-10, 40}, {-10, -40}, {10, -40}}), Line(origin = {75, 0}, points = {{5, 0}, {-5, 0}})}));
      end IdealGNSS;

      model IdealAsset
        extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
        Modelica.Blocks.Interfaces.RealOutput q[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      
      equation
        frame_a.f = {0, 0, 0};
        frame_a.t = {0, 0, 0};
        q = Modelica.Mechanics.MultiBody.Frames.Quaternions.from_T(frame_a.R.T);
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "q"), Line(origin = {85, 0}, points = {{-15, 0}, {15, 0}})}, coordinateSystem(grid = {2, 0})));
      end IdealAsset;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end IdealSensors;

    package RealSensors

      model RealGyroscope "Implementation of a real gyroscope, affected by startup random bias, bias instability (Rate Random Walk) and Normal Noise (Angle Random Walk)"
         extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
         
        parameter Integer samplePeriodMs(min = 1) "Sample period in milliseconds"  annotation(
          Dialog(group = "Sampling and noise"));
        parameter Boolean noisy = false "= true, if output should be superimposed with noise" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Sampling and noise"));
        parameter Boolean biased = false "= true, if output should be biased" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Bias"));
        parameter Boolean limited = false "= true, if output is limited" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Limiting and quantization"));
        parameter Boolean quantized = false "= true, if output quantization effects included" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(enable = limited, group = "Limiting and quantization"));
        parameter Types.AngularVelocity bias[3] "Measurement bias for each axis" annotation(
          Dialog(enable = biased, group = "Bias"));
        parameter Types.AngularVelocity rate_max "Angular velocity measurement upper limit" annotation(
          Dialog(enable = limited, group = "Limiting and quantization"));
        parameter Types.AngularVelocity rate_min = -rate_max "Angular velocity measurement lower limit" annotation(
          Dialog(enable = limited, group = "Limiting and quantization"));
      parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
          Dialog(enable = limited and quantized, group = "Limiting and quantization"));
        parameter Integer fixedLocalSeed[3] = {10, 100, 1000} "Local seed for each of the accelerometer axes" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
        parameter Types.AngularVelocity sigmaARW "Angular random walk standard deviation" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
        parameter SI.AngularAcceleration sigmaRRW(displayUnit = "deg/s2") "Rate random walk standard deviation" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
      
        RocketControl.Components.Sensors.Internal.ADeffects sampleX(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaARW, sigmaRW = sigmaRRW, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[1]), bias = bias[1], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = rate_max, yMin = rate_min) annotation(
          Placement(visible = true, transformation(origin = {0, 40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleY(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaARW, sigmaRW = sigmaRRW, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[2]), bias = bias[2], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = rate_max, yMin = rate_min) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleZ(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaARW, sigmaRW = sigmaRRW, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[3]), bias = bias[3], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = rate_max, yMin = rate_min) annotation(
          Placement(visible = true, transformation(origin = {0, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      RocketControl.Components.Sensors.IdealSensors.IdealGyroscope idealGyroscope annotation(
          Placement(visible = true, transformation(origin = {-58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          
          Modelica.Blocks.Interfaces.RealOutput w[3](each final unit="rad/s", each final quantity="AngularVelocity",each displayUnit="deg/s") annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(sampleX.y, w[1]) annotation(
          Line(points = {{6, 40}, {40, 40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
        connect(sampleY.y, w[2]) annotation(
          Line(points = {{106, 0}, {6, 0}}, color = {0, 0, 127}));
        connect(sampleZ.y, w[3]) annotation(
          Line(points = {{6, -40}, {40, -40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
      connect(idealGyroscope.w[1], sampleX.u) annotation(
          Line(points = {{-47, 0}, {-30, 0}, {-30, 40}, {-8, 40}}, color = {0, 0, 127}));
      connect(idealGyroscope.w[2], sampleY.u) annotation(
          Line(points = {{-47, 0}, {-8, 0}}, color = {0, 0, 127}));
      connect(idealGyroscope.w[3], sampleZ.u) annotation(
          Line(points = {{-47, 0}, {-32, 0}, {-32, -40}, {-8, -40}}, color = {0, 0, 127}));
      connect(frame_a, idealGyroscope.frame_a) annotation(
          Line(points = {{-100, 0}, {-68, 0}}));
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "w")}),
          Diagram);
      end RealGyroscope;

      model RealAccelerometer
          extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
        parameter Integer samplePeriodMs(min = 1) "Sample period in milliseconds"  annotation(
          Dialog(group = "Sampling and noise"));
        parameter Boolean noisy = false "= true, if output should be superimposed with noise" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Sampling and noise"));
        parameter Boolean biased = false "= true, if output should be biased" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Bias"));
        parameter Boolean limited = false "= true, if output is limited" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Limiting and quantization"));
        parameter Boolean quantized = false "= true, if output quantization effects included" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(enable = limited, group = "Limiting and quantization"));
        parameter SI.Acceleration bias[3] "Measurement bias for each axis" annotation(
          Dialog(enable = biased, group = "Bias"));
        parameter SI.Acceleration acc_max "Acceleration measurement upper limit" annotation(
          Dialog(enable = limited, group = "Limiting and quantization"));
        parameter SI.Acceleration acc_min = -acc_max "Acceleration measurement lower limit" annotation(
          Dialog(enable = limited, group = "Limiting and quantization"));
      parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
          Dialog(enable = limited and quantized, group = "Limiting and quantization"));
        parameter Integer fixedLocalSeed[3] = {10, 100, 1000} "Local seed for each of the accelerometer axes" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
        parameter SI.Acceleration sigmaNoise "Noise standard deviation" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
        parameter SI.Jerk sigmaBiasInstability "Bias instability standard deviation" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
      
      
        
        RocketControl.Components.Sensors.IdealSensors.IdealAccelerometer idealAccelerometer annotation(
          Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleX(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasInstability, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[1]), bias = bias[1], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = acc_max, yMin = acc_min) annotation(
          Placement(visible = true, transformation(origin = {0, 40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleY(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasInstability, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[2]), bias = bias[1], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = acc_max, yMin = acc_min) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleZ(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasInstability, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[3]), bias = bias[1], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = acc_max, yMin = acc_min) annotation(
          Placement(visible = true, transformation(origin = {0, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
          
              Modelica.Blocks.Interfaces.RealOutput a[3](each final unit="m/s2", each final quantity="Acceleration") annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(frame_a, idealAccelerometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-70, 0}}));
        connect(idealAccelerometer.a[1], sampleX.u) annotation(
          Line(points = {{-50, 0}, {-29, 0}, {-29, 40}, {-7, 40}}, color = {0, 0, 127}));
        connect(idealAccelerometer.a[2], sampleY.u) annotation(
          Line(points = {{-50, 0}, {-8, 0}}, color = {0, 0, 127}));
        connect(idealAccelerometer.a[3], sampleZ.u) annotation(
          Line(points = {{-50, 0}, {-28, 0}, {-28, -40}, {-8, -40}}, color = {0, 0, 127}));
        connect(sampleX.y, a[1]) annotation(
          Line(points = {{6, 40}, {40, 40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
        connect(sampleY.y, a[2]) annotation(
          Line(points = {{6, 0}, {106, 0}}, color = {0, 0, 127}));
        connect(sampleZ.y, a[3]) annotation(
          Line(points = {{6, -40}, {40, -40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "m/s^2"), Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name")}));
      end RealAccelerometer;

      model RealMagnetometer "Magnetometer sensor model with bias, gaussian noise and quantization effects"
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
        parameter Integer samplePeriodMs(min = 1) "Sample period in milliseconds"  annotation(
          Dialog(group = "Sampling and noise"));
        parameter Boolean noisy = false "= true, if output should be superimposed with noise" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Sampling and noise"));
        parameter Boolean biased = false "= true, if output should be biased" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Bias"));
        parameter Boolean limited = false "= true, if output is limited" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Limiting and quantization"));
        parameter Boolean quantized = false "= true, if output quantization effects included" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(enable = limited, group = "Limiting and quantization"));
          parameter Types.MagneticFluxDensity bias "Measurement bias" annotation(
          Dialog(enable = biased, group = "Bias"));
        parameter SI.Angle misalignement[3] "Magnetic vector measurement miasalignment" annotation(
          Dialog(enable = biased, group = "Bias"));
        parameter Types.MagneticFluxDensity b_max "Magnetic field measurement upper limit" annotation(
          Dialog(enable = limited, group = "Limiting and quantization"));
        parameter Types.MagneticFluxDensity b_min = -b_max "Magnetic field measurement lower limit" annotation(
          Dialog(enable = limited, group = "Limiting and quantization"));
      parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
          Dialog(enable = limited and quantized, group = "Limiting and quantization"));
        parameter Integer fixedLocalSeed[3] = {10, 100, 1000} "Local seed for each of the axes" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
        parameter Types.MagneticFluxDensity sigmaNoise "Noise standard deviation" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
      
        RocketControl.Components.Sensors.Internal.ADeffects sampleX(samplePeriodMs = samplePeriodMs, bias = 0, biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[1], sigma = sigmaNoise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, yMax = b_max, yMin = b_min) annotation(
          Placement(visible = true, transformation(origin = {16, 40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleY(samplePeriodMs = samplePeriodMs, bias = 0, biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[2], sigma = sigmaNoise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, yMax = b_max, yMin = b_min) annotation(
          Placement(visible = true, transformation(origin = {16, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleZ(samplePeriodMs = samplePeriodMs, bias = 0, biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[3], sigma = sigmaNoise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, yMax = b_max, yMin = b_min) annotation(
          Placement(visible = true, transformation(origin = {16, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealSensors.IdealMagnetometer idealMagnetometer annotation(
          Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput b[3](each final unit="T", each final quantity="MagneticFluxDensity",each displayUnit="nT") annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      protected
        final parameter Modelica.Mechanics.MultiBody.Frames.Orientation R_mis = Modelica.Mechanics.MultiBody.Frames.axesRotations({3, 2, 1}, misalignement, {0, 0, 0}) annotation(
          Evaluate = true);
          Types.MagneticFluxDensity b_mis[3];
        Types.NanoTesla b_biased[3];
      equation
        b_biased = idealMagnetometer.b + idealMagnetometer.b * bias / norm(idealMagnetometer.b);
        b_mis = Modelica.Mechanics.MultiBody.Frames.resolve2(R_mis, b_biased);
        sampleX.u = b_mis[1];
        sampleY.u = b_mis[2];
        sampleZ.u = b_mis[3];
        connect(frame_a, idealMagnetometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-70, 0}}));
        connect(sampleX.y, b[1]) annotation(
          Line(points = {{23, 40}, {40, 40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
        connect(sampleY.y, b[2]) annotation(
          Line(points = {{23, 0}, {106, 0}}, color = {0, 0, 127}));
        connect(sampleZ.y, b[3]) annotation(
          Line(points = {{23, -40}, {40, -40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "rad/s")}),
          Diagram);
      end RealMagnetometer;

      model RealBarometer
         extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
         
        parameter Integer samplePeriodMs(min = 1) "Sample period in milliseconds"  annotation(
          Dialog(group = "Sampling and noise"));
        parameter Boolean noisy = false "= true, if output should be superimposed with noise" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Sampling and noise"));
        parameter Boolean biased = false "= true, if output should be biased" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Bias"));
        parameter Boolean limited = false "= true, if output is limited" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Limiting and quantization"));
        parameter Boolean quantized = false "= true, if output quantization effects included" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(enable = limited, group = "Limiting and quantization"));
        parameter SI.Pressure bias "Measurement bias" annotation(
          Dialog(enable = biased, group = "Bias"));
        parameter SI.Pressure p_max "Pressure measurement upper limit" annotation(
          Dialog(enable = limited, group = "Limiting and quantization"));
        parameter SI.Pressure p_min = -p_max "Pressure measurement lower limit" annotation(
          Dialog(enable = limited, group = "Limiting and quantization"));
      parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
          Dialog(enable = limited and quantized, group = "Limiting and quantization"));
        parameter Integer fixedLocalSeed = 10 "Local seed" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
        parameter SI.Pressure sigmaNoise "Noise standard deviation" annotation(
          Dialog(enable = noisy, group = "Sampling and noise"));
      
        RocketControl.Components.Sensors.IdealSensors.IdealBarometer idealBarometer annotation(
          Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sample(samplePeriodMs = samplePeriodMs, noise(sigma = sigmaNoise, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed), bias = bias, biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = p_max, yMin = p_min) annotation(
          Placement(visible = true, transformation(origin = {40, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
       Modelica.Blocks.Interfaces.RealOutput p(final unit="Pa", final quantity="Pressure", displayUnit="kPa") annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      
      equation
        connect(frame_a, idealBarometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-60, 0}}));
        connect(idealBarometer.p, sample.u) annotation(
          Line(points = {{-40, 0}, {32, 0}}, color = {0, 0, 127}));
        connect(sample.y, p) annotation(
          Line(points = {{46, 0}, {106, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "Pa"), Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Line(origin = {81, 0}, points = {{-11, 0}, {11, 0}})}));
      end RealBarometer;

      model RealGNSS
        extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      
         parameter Integer samplePeriodMs(min = 1) "Sample period in milliseconds"  annotation(
          Dialog(group = "Sampling and noise"));
        parameter Boolean noisy = false "= true, if output should be superimposed with noise" annotation(
          Evaluate = true,
          choices(checkBox = true),
          Dialog(group = "Sampling and noise"));
      
        parameter SI.Position sigmaNoise_xy "Position noise standard deviation on the horizontal plane" annotation(
          Evaluate = true,
          Dialog(group = "Sampling and noise"));
        parameter SI.Position sigmaNoise_z "Position noise standard deviation on the vertical plane" annotation(
          Evaluate = true,
          Dialog(group = "Sampling and noise"));
        parameter SI.Velocity sigmaNoise_vxy "Velocity noise standard deviation on the horizontal plane" annotation(
          Evaluate = true,
          Dialog(group = "Sampling and noise"));
        parameter SI.Velocity sigmaNoise_vz "Velocity noise standard deviation on the vertical plane" annotation(
          Evaluate = true,
          Dialog(group = "Sampling and noise"));
      
        parameter SI.Position sin_error_amplitude[3] = {0, 0, 0}  annotation(
          Dialog(group = "Bias"));
        parameter SI.Frequency sin_error_freq  annotation(
          Dialog(group = "Bias"));
        parameter SI.Angle sin_error_phase[3](each displayUnit = "deg") = {0, 0, 0}  annotation(
          Dialog(group = "Bias"));
        parameter Integer fixedLocalSeed[3] = {10, 100, 1000} "Local seed for each of the accelerometer axes" annotation(
          Evaluate = true,
          Dialog(group = "Sampling and noise"));
      
      
        Modelica.Blocks.Interfaces.RealOutput x[3](each final quantity = "Position", each final unit = "m") annotation(
          Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = samplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {-94, -94}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects samplePX(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[1], sigma = sigmaNoise_xy, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
          Placement(visible = true, transformation(origin = {60, 100}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects samplePY(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[2], sigma = sigmaNoise_xy, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
          Placement(visible = true, transformation(origin = {60, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects samplePZ(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[3], sigma = sigmaNoise_z, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
          Placement(visible = true, transformation(origin = {60, 20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleVX(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[1] + 1, sigma = sigmaNoise_vxy, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
          Placement(visible = true, transformation(origin = {60, -20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleVY(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[3] + 1, sigma = sigmaNoise_vz, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
          Placement(visible = true, transformation(origin = {60, -100}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.ADeffects sampleVZ(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[2] + 1, sigma = sigmaNoise_vxy, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
          Placement(visible = true, transformation(origin = {60, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput v[3](each quantity = "Velocity") annotation(
          Placement(visible = true, transformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorAdd vectorAdd annotation(
          Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorSine vectorSine(A = sin_error_amplitude, f = sin_error_freq, phase = sin_error_phase) annotation(
          Placement(visible = true, transformation(origin = {-40, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorSine vectorSine1(A = sin_error_amplitude * 2 * pi * sin_error_freq, f = sin_error_freq, phase = sin_error_phase + from_deg({90, 90, 90})) annotation(
          Placement(visible = true, transformation(origin = {-68, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorAdd vectorAdd1 annotation(
          Placement(visible = true, transformation(origin = {10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealSensors.IdealGNSS idealGNSS annotation(
          Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(vectorAdd.vc[1], samplePX.u) annotation(
          Line(points = {{11, 60}, {28, 60}, {28, 100}, {52, 100}}, color = {0, 0, 127}));
        connect(vectorAdd.vc[2], samplePY.u) annotation(
          Line(points = {{12, 60}, {52, 60}}, color = {0, 0, 127}));
        connect(vectorAdd.vc[3], samplePZ.u) annotation(
          Line(points = {{12, 60}, {28, 60}, {28, 20}, {52, 20}}, color = {0, 0, 127}));
        connect(vectorSine.y, vectorAdd.v1) annotation(
          Line(points = {{-28, 90}, {-24, 90}, {-24, 64}, {-12, 64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorSine1.y, vectorAdd1.v1) annotation(
          Line(points = {{-57, -32}, {-24, -32}, {-24, -56}, {-2, -56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorAdd1.vc[1], sampleVX.u) annotation(
          Line(points = {{21, -60}, {28, -60}, {28, -20}, {52, -20}}, color = {0, 0, 127}));
        connect(vectorAdd1.vc[2], sampleVY.u) annotation(
          Line(points = {{21, -60}, {52, -60}}, color = {0, 0, 127}));
        connect(vectorAdd1.vc[3], sampleVZ.u) annotation(
          Line(points = {{21, -60}, {28, -60}, {28, -100}, {52, -100}}, color = {0, 0, 127}));
        connect(samplePX.y, x[1]) annotation(
          Line(points = {{66, 100}, {80, 100}, {80, 60}, {110, 60}}, color = {0, 0, 127}));
        connect(samplePY.y, x[2]) annotation(
          Line(points = {{66, 60}, {110, 60}}, color = {0, 0, 127}));
        connect(samplePZ.y, x[3]) annotation(
          Line(points = {{66, 20}, {80, 20}, {80, 60}, {110, 60}}, color = {0, 0, 127}));
        connect(sampleVX.y, v[1]) annotation(
          Line(points = {{66, -20}, {80, -20}, {80, -60}, {110, -60}}, color = {0, 0, 127}));
        connect(sampleVY.y, v[2]) annotation(
          Line(points = {{66, -60}, {110, -60}}, color = {0, 0, 127}));
        connect(sampleVZ.y, v[3]) annotation(
          Line(points = {{66, -100}, {80, -100}, {80, -60}, {110, -60}}, color = {0, 0, 127}));
        connect(frame_a, idealGNSS.frame_a) annotation(
          Line(points = {{-100, 0}, {-80, 0}}));
        connect(idealGNSS.x, vectorAdd.v2) annotation(
          Line(points = {{-58, 4}, {-42, 4}, {-42, 56}, {-12, 56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGNSS.v, vectorAdd1.v2) annotation(
          Line(points = {{-58, -4}, {-40, -4}, {-40, -64}, {-2, -64}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "GNSS"), Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Line(origin = {90, 0}, points = {{10, 40}, {-10, 40}, {-10, -40}, {10, -40}}), Line(origin = {75, 0}, points = {{5, 0}, {-5, 0}})}));
      end RealGNSS;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end RealSensors;

    package Parts
      model GyroBMX180
        extends RealSensors.RealGyroscope(bias = {0.00174532925199433, -0.0008726646259971648, -0.0003490658503988659}, bits = 16, rate_max = 4.363323129985824, sigmaARW = 0.008726646259971648, sigmaRRW = 0.0174532925199433, noisy = true, biased = true, limited = true, quantized = true);
       
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end GyroBMX180;

      model AccBMX180
      extends RealSensors.RealAccelerometer(acc_max = 156.96, bias = {0.02, 0.04, -0.03}, bits = 16,  sigmaBiasInstability = 0.01, sigmaNoise = 0.1, noisy = true, biased = true, limited = true, quantized = true);
      
      equation

        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end AccBMX180;

      model MagBMX160
      extends RealSensors.RealMagnetometer(b_max = 2500000 * 1e-9, bias = 250 * 1e-9, bits = 16, misalignement = {from_deg(0.2), from_deg(-0.1), from_deg(0.3)}, sigmaNoise = 1.000000000000001e-08, noisy = true, biased = true, limited = true, quantized = true);
       
      equation

        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end MagBMX160;

      model BaroMS5803
      extends RealSensors.RealBarometer(bias(displayUnit = "Pa") = 300, bits = 24, p_max(displayUnit = "Pa") = 110000, p_min(displayUnit = "Pa") = 999.9999999999999, sigmaNoise(displayUnit = "Pa") = 50, noisy = true, biased = true, limited = true, quantized = true);
       
      equation

        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end BaroMS5803;

      model GNSSUbloxM7N
      extends RealSensors.RealGNSS(sigmaNoise_vxy = 3, sigmaNoise_vz = 5, sigmaNoise_xy = 15, sigmaNoise_z = 30, sin_error_amplitude = {30, 30, 50}, sin_error_freq = 0.005, sin_error_phase = from_deg({20, 30, 40}), noisy=true);
       
      equation

        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end GNSSUbloxM7N;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Parts;

    package Conversions
      model Altimeter
        import Modelica.Constants.R;
        parameter SI.Position ref_altitude = world.altitude_0;
        parameter SI.Temperature ref_temperature = atmosphere.temperature(ref_altitude);
        parameter SI.Pressure ref_pressure = atmosphere.pressure(ref_altitude);
        parameter RocketControl.Types.LapseRate lapse_rate = 0.0065;
        parameter SI.Acceleration g0 = 9.80665;
        parameter SI.MolarMass M = 0.0289644;
        parameter SI.SpecificHeatCapacity Rs = R / M;
        final parameter Real n = g0 / (Rs * lapse_rate);
        final parameter SI.Temperature msl_temperature = ref_temperature + ref_altitude * lapse_rate;
        final parameter SI.Pressure msl_pressure = ref_pressure / (1 - lapse_rate * ref_altitude / msl_temperature) ^ n;
        Modelica.Blocks.Interfaces.RealOutput h annotation(
          Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput p annotation(
          Placement(visible = true, transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      protected
        outer RocketControl.World.MyWorld world;
        outer RocketControl.World.Atmosphere atmosphere;
      equation
        h = msl_temperature / lapse_rate * (1 - (p / msl_pressure) ^ (1 / n));
        annotation(
          Icon(graphics = {Text(origin = {-3, 5}, extent = {{-83, 85}, {83, -85}}, textString = "ALT")}));
      end Altimeter;

      model Variometer
        import Modelica.Constants.R;
        parameter SI.Position ref_altitude = world.altitude_0;
        parameter SI.Temperature ref_temperature = atmosphere.temperature(ref_altitude);
        parameter SI.Pressure ref_pressure = atmosphere.pressure(ref_altitude);
        parameter RocketControl.Types.LapseRate lapse_rate = 0.0065;
        parameter SI.Acceleration g0 = 9.80665;
        parameter SI.MolarMass M = 0.0289644;
        parameter SI.SpecificHeatCapacity Rs = R / M;
        final parameter Real n = g0 / (Rs * lapse_rate);
        final parameter SI.Temperature msl_temperature = ref_temperature + ref_altitude * lapse_rate;
        final parameter SI.Pressure msl_pressure = ref_pressure / (1 - lapse_rate * ref_altitude / msl_temperature) ^ n;
        Modelica.Blocks.Interfaces.RealOutput vs annotation(
          Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput p annotation(
          Placement(visible = true, transformation(origin = {-106, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput dp annotation(
          Placement(visible = true, transformation(origin = {-106, -42}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      protected
        outer RocketControl.World.MyWorld world;
        outer RocketControl.World.Atmosphere atmosphere;
      equation
        vs = -msl_temperature * dp * (p / msl_pressure) ^ (1 / n) / (lapse_rate * n * p);
        annotation(
          Icon(graphics = {Text(origin = {-3, 5}, extent = {{-83, 85}, {83, -85}}, textString = "VAR")}));
      end Variometer;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Conversions;

    package Internal
      block ADeffects "Sample with (simulated) Analog-Digital converter effects including noise"
              extends Modelica.Clocked.RealSignals.Interfaces.PartialSISOSampler;
              parameter Integer samplePeriodMs(min = 1) "Sample period in milliseconds"  annotation(
          Dialog(group = "Sampling and noise"));
              parameter Boolean noisy = false "= true, if output should be superimposed with noise" annotation(
                Evaluate = true,
                choices(checkBox = true),
                Dialog(group = "Sampling and noise"));
              parameter Boolean biased = false "= true, if output should be biased" annotation(
                Evaluate = true,
                choices(checkBox = true),
                Dialog(group = "Bias"));
              parameter Boolean limited = false "= true, if output is limited" annotation(
                Evaluate = true,
                choices(checkBox = true),
                Dialog(group = "Limiting and quantization"));
              parameter Boolean quantized = false "= true, if output quantization effects included" annotation(
                Evaluate = true,
                choices(checkBox = true),
                Dialog(enable = limited, group = "Limiting and quantization"));
              parameter Real yMax = 1 "Upper limit of output (if limited = true)" annotation(
                Dialog(enable = limited, group = "Limiting and quantization"));
              parameter Real yMin = -yMax "Lower limit of output (if limited = true)" annotation(
                Dialog(enable = limited, group = "Limiting and quantization"));
              parameter Integer bits(min = 1) = 8 "Number of bits of quantization (if quantized = true)" annotation(
                Dialog(enable = limited and quantized, group = "Limiting and quantization"));
              parameter Real bias "Signal bias" annotation(
                Dialog(enable = biased, group = "Bias"));
              replaceable RocketControl.Components.Sensors.Internal.Noise.ClockedNormalNoise noise if noisy constrainedby Modelica.Clocked.RealSignals.Interfaces.PartialNoise "Noise model" annotation(
                 Dialog(enable = noisy, group = "Sampling and noise"),
                 Placement(transformation(extent = {{-54, -6}, {-42, 6}})));
              Modelica.Clocked.RealSignals.Sampler.Utilities.Internal.Limiter limiter(uMax = yMax, uMin = yMin) if limited annotation(
                Placement(visible = true, transformation(extent = {{22, -8}, {38, 8}}, rotation = 0)));
              Modelica.Clocked.RealSignals.Sampler.Utilities.Internal.Quantization quantization(quantized = quantized, yMax = yMax, yMin = yMin, bits = bits) if quantized and limited annotation(
                Placement(visible = true, transformation(extent = {{60, -8}, {76, 8}}, rotation = 0)));
              Modelica.Blocks.Math.Add add if biased annotation(
                Placement(visible = true, transformation(origin = {-12, -1.33227e-15}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
              Modelica.Blocks.Sources.Constant bias_k(k = bias) if biased annotation(
                Placement(visible = true, transformation(origin = {-48, -30}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = samplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {-88, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Clocked.RealSignals.Sampler.SampleClocked sample1 annotation(
          Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
            protected
              Modelica.Blocks.Interfaces.RealInput uFeedthrough1 if not noisy annotation(
                Placement(transformation(extent = {{-58, 12}, {-42, 28}})));
              Modelica.Blocks.Interfaces.RealInput uFeedthrough2 if not biased annotation(
                Placement(visible = true, transformation(extent = {{-20, 12}, {-4, 28}}, rotation = 0), iconTransformation(extent = {{-26, 12}, {-10, 28}}, rotation = 0)));
              Modelica.Blocks.Interfaces.RealInput uFeedthrough3 if not limited annotation(
                Placement(visible = true, transformation(extent = {{20, 12}, {36, 28}}, rotation = 0), iconTransformation(extent = {{-26, 12}, {-10, 28}}, rotation = 0)));
              Modelica.Blocks.Interfaces.RealInput uFeedthrough4 if not quantized or not limited annotation(
                Placement(visible = true, transformation(extent = {{58, 12}, {74, 28}}, rotation = 0), iconTransformation(extent = {{12, 12}, {28, 28}}, rotation = 0)));
              Modelica.Blocks.Interfaces.RealOutput y1 "Connector with a Real output signal" annotation(
                Placement(transformation(extent = {{-61, -1}, {-59, 1}})));
              Modelica.Blocks.Interfaces.RealOutput y3 annotation(
                Placement(visible = true, transformation(extent = {{11, -1}, {13, 1}}, rotation = 0), iconTransformation(extent = {{-35, -1}, {-33, 1}}, rotation = 0)));
              Modelica.Blocks.Interfaces.RealOutput y4 annotation(
                Placement(visible = true, transformation(extent = {{49, -1}, {51, 1}}, rotation = 0), iconTransformation(extent = {{3, -1}, {5, 1}}, rotation = 0)));
              Modelica.Blocks.Interfaces.RealOutput y5 annotation(
                Placement(visible = true, transformation(extent = {{87, -1}, {89, 1}}, rotation = 0), iconTransformation(extent = {{41, -1}, {43, 1}}, rotation = 0)));
              Modelica.Blocks.Interfaces.RealOutput y2 annotation(
                Placement(visible = true, transformation(extent = {{-31, -1}, {-29, 1}}, rotation = 0), iconTransformation(extent = {{-35, -1}, {-33, 1}}, rotation = 0)));
            equation
              connect(y1, uFeedthrough1) annotation(
                Line(points = {{-50, 20}, {-58, 20}, {-58, 0}, {-60, 0}}, color = {0, 0, 127}));
              connect(y1, noise.u) annotation(
                Line(points = {{-60, 0}, {-55.2, 0}}, color = {0, 0, 127})); 
              connect(sample1.y, y1) annotation(
          Line(points = {{-74, 0}, {-60, 0}}, color = {0, 0, 127}));
          
              connect(y3, limiter.u) annotation(
                Line(points = {{12, 0}, {20.4, 0}}, color = {0, 0, 127}));
              connect(y3, uFeedthrough3) annotation(
                Line(points = {{12, 0}, {16, 0}, {16, 20}, {28, 20}}, color = {0, 0, 127}));
              connect(limiter.y, y4) annotation(
                Line(points = {{38.8, 0}, {50, 0}}, color = {0, 0, 127}));
              connect(y4, quantization.u) annotation(
                Line(points = {{50, 0}, {58.4, 0}}, color = {0, 0, 127}));
              connect(y4, uFeedthrough4) annotation(
                Line(points = {{50, 0}, {54, 0}, {54, 20}, {66, 20}}, color = {0, 0, 127}));
              connect(quantization.y, y5) annotation(
                Line(points = {{76.8, 0}, {88, 0}}, color = {0, 0, 127}));
              connect(uFeedthrough4, y5) annotation(
                Line(points = {{66, 20}, {84, 20}, {84, 0}, {88, 0}}, color = {0, 0, 127}));
              connect(uFeedthrough3, y4) annotation(
                Line(points = {{28, 20}, {46, 20}, {46, 0}, {50, 0}}, color = {0, 0, 127}));
              connect(y5, y) annotation(
                Line(points = {{88, 0}, {110, 0}}, color = {0, 0, 127}));
              connect(noise.y, y2) annotation(
                Line(points = {{-42, 0}, {-30, 0}}, color = {0, 0, 127}));
              connect(uFeedthrough1, y2) annotation(
                Line(points = {{-50, 20}, {-36, 20}, {-36, 0}, {-30, 0}}, color = {0, 0, 127}));
              connect(y2, uFeedthrough2) annotation(
                Line(points = {{-30, 0}, {-28, 0}, {-28, 20}, {-12, 20}}, color = {0, 0, 127}));
              connect(uFeedthrough2, y3) annotation(
                Line(points = {{-12, 20}, {4, 20}, {4, 0}, {12, 0}}, color = {0, 0, 127}));
              connect(add.u1, y2) annotation(
                Line(points = {{-22, 4}, {-26, 4}, {-26, 0}, {-30, 0}}, color = {0, 0, 127}));
              connect(add.y, y3) annotation(
                Line(points = {{-4, 0}, {12, 0}}, color = {0, 0, 127}));
              connect(bias_k.y, add.u2) annotation(
                Line(points = {{-41, -30}, {-32, -30}, {-32, -4}, {-22, -4}}, color = {0, 0, 127}));
      connect(u, sample1.u) annotation(
          Line(points = {{-120, 0}, {-88, 0}}, color = {0, 0, 127}));
      connect(periodicClock1.y, sample1.clock) annotation(
          Line(points = {{-82, -60}, {-80, -60}, {-80, -8}}, color = {175, 175, 175}));
              annotation(
                Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.06), graphics = {Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{0, -22}, {-6, -38}, {6, -38}, {0, -22}}), Line(points = {{0, -100}, {0, -38}}, color = {192, 192, 192}), Line(points = {{-40, -72}, {40, -72}}, color = {192, 192, 192}), Polygon(origin = {48, -72}, rotation = -90, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{0, 8}, {-6, -8}, {6, -8}, {0, 8}}), Line(points = {{-30, -92}, {-10, -92}, {-10, -72}, {10, -72}, {10, -52}, {30, -52}}, color = {0, 0, 127}), Text(lineColor = {0, 0, 255}, extent = {{-150, 90}, {150, 50}}, textString = "%name")}));
            end ADeffects;

      package Noise
        block ClockedNormalNoise "Noise generator with normal distribution"
          import distribution = Modelica.Math.Distributions.Normal.quantile;
          extends PartialClockedNoise;
          // Main dialog menu
          parameter Real mu = 0 "Expectation (mean) value of the normal distribution" annotation(
            Dialog(enable = enableNoise));
          parameter Real sigma = 1 "Standard deviation of the normal distribution" annotation(
            Dialog(enable = enableNoise));
        equation
// Draw random number at sample times
          when Clock() then
            r = distribution(r_raw, mu, sigma);
          end when;
          annotation(
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Text(visible = enableNoise, extent = {{-66, 92}, {94, 66}}, textColor = {175, 175, 175}, textString = "mu=%mu"), Text(visible = enableNoise, extent = {{-70, -68}, {94, -96}}, textColor = {175, 175, 175}, textString = "sigma=%sigma")}),
            Documentation(info = "<html>
        <p>
        A summary of the common properties of the noise blocks is provided in the documentation of package
        <a href=\"modelica://Modelica.Blocks.Noise\">Blocks.Noise</a>.
        This NormalNoise block generates reproducible, random noise at its output according to a normal distribution.
        This means that random values are normally distributed with expectation value mu and standard deviation sigma.
        (see example <a href=\"modelica://Modelica.Blocks.Examples.Noise.NormalNoiseProperties\">Examples.Noise.NormalNoiseProperties</a>).
        By default, two or more instances produce different, uncorrelated noise at the same time instant.
        The block can only be used if on the same or a higher hierarchical level,
        model <a href=\"modelica://Modelica.Blocks.Noise.GlobalSeed\">Blocks.Noise.GlobalSeed</a>
        is dragged to provide global settings for all instances.
        </p>
        </html>", revisions = "<html>
        <table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
        <tr><th>Date</th> <th align=\"left\">Description</th></tr>
        
        <tr><td> June 22, 2015 </td>
          <td>
        
        <table border=\"0\">
        <tr><td>
               <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
        </td><td valign=\"bottom\">
               Initial version implemented by
               A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
               <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
        </td></tr></table>
        </td></tr>
        
        </table>
        </html>"));
        end ClockedNormalNoise;

        partial block PartialClockedNoise "Partial noise generator"
          import generator = Modelica.Math.Random.Generators.Xorshift128plus;
          import Modelica.Math.Random.Utilities.automaticLocalSeed;
          extends Modelica.Clocked.RealSignals.Interfaces.PartialNoise;
          // Advanced dialog menu: Noise generation
          parameter Boolean enableNoise = globalSeed.enableNoise "= true: y = u + noise, otherwise y = u" annotation(
            choices(checkBox = true),
            Dialog(tab = "Advanced", group = "Noise generation"));
          // Advanced dialog menu: Initialization
          parameter Boolean useGlobalSeed = true "= true: use global seed, otherwise ignore it" annotation(
            choices(checkBox = true),
            Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise));
          parameter Boolean useAutomaticLocalSeed = true "= true: use automatic local seed, otherwise use fixedLocalSeed" annotation(
            choices(checkBox = true),
            Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise));
          parameter Integer fixedLocalSeed = 1 "Local seed (any Integer number)" annotation(
            Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise and not useAutomaticLocalSeed));
          final parameter Integer localSeed(fixed = false) "The actual localSeed";
        protected
          outer Modelica.Blocks.Noise.GlobalSeed globalSeed "Definition of global seed via inner/outer";
          parameter Integer actualGlobalSeed = if useGlobalSeed then globalSeed.seed else 0 "The global seed, which is actually used";
          parameter Boolean generateNoise = enableNoise and globalSeed.enableNoise "= true, if noise shall be generated, otherwise no noise";
          // Declare state and random number variables
          Integer state[generator.nState](start = generator.initialState(localSeed, actualGlobalSeed)) "Internal state of random number generator";
          Real r "Random number according to the desired distribution";
          Real r_raw "Uniform random number in the range (0,1]";
        initial equation
          localSeed = if useAutomaticLocalSeed then automaticLocalSeed(getInstanceName()) else fixedLocalSeed;
//   pre(state) = generator.initialState(localSeed, actualGlobalSeed);
//   r_raw = generator.random(pre(state));
        equation
// Draw random number at sample times
//  when firstTick(clock) then
//    r_raw = generator.random(previous(state));
//  end when;
          when Clock() then
            (r_raw, state) = generator.random(pre(state));
// Generate noise if requested
            y = if not generateNoise then u else u + r;
          end when;
          annotation(
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-81, -17}, {-67, -17}, {-67, -1}, {-59, -1}, {-59, -49}, {-51, -49}, {-51, -27}, {-43, -27}, {-43, 57}, {-35, 57}, {-35, 25}}, color = {0, 0, 127}, pattern = LinePattern.Dot), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{40, 19}, {46, 13}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-54, -23}, {-48, -29}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-18, -41}, {-12, -47}}, endAngle = 360), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{91, -22}, {69, -14}, {69, -30}, {91, -22}}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-62, -47}, {-56, -53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-28, -15}, {-22, -21}}, endAngle = 360), Line(points = {{-90, -23}, {82, -23}}, color = {192, 192, 192}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{14, 9}, {20, 3}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{4, -1}, {10, -7}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-8, 39}, {-2, 33}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{48, -47}, {54, -53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-2, 53}, {4, 47}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{30, 53}, {36, 47}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-46, 59}, {-40, 53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{20, -19}, {26, -25}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-84, -13}, {-78, -19}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-38, -33}, {-32, -39}}, endAngle = 360), Line(points = {{-35, 25}, {-35, -35}, {-25, -35}, {-25, -17}, {-15, -17}, {-15, -45}, {-5, -45}, {-5, 37}, {1, 37}, {1, 51}, {7, 51}, {7, -5}, {17, -5}, {17, 7}, {23, 7}, {23, -23}, {33, -23}, {33, 49}, {43, 49}, {43, 15}, {51, 15}, {51, -51}, {61, -51}}, color = {0, 0, 127}, pattern = LinePattern.Dot), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-70, 3}, {-64, -3}}, endAngle = 360), Line(points = {{-81, 78}, {-81, -90}}, color = {192, 192, 192}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-81, 90}, {-89, 68}, {-73, 68}, {-81, 90}})}),
            Documentation(info = "<html>
        <p>
        Partial base class of noise generators defining the common features
        of noise blocks.
        </p>
        </html>", revisions = "<html>
        <table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
        <tr><th>Date</th> <th align=\"left\">Description</th></tr>
        
        <tr><td> June 22, 2015 </td>
          <td>
        
        <table border=\"0\">
        <tr><td>
               <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
        </td><td valign=\"bottom\">
               Initial version implemented by
               A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
               <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
        </td></tr></table>
        </td></tr>
        
        </table>
        </html>"));
        end PartialClockedNoise;

        model ClockedSensorNoise
          extends Modelica.Clocked.RealSignals.Interfaces.PartialNoise;
          parameter Boolean enableNoise = globalSeed.enableNoise "= true: y = u + noise, otherwise y = u" annotation(
            choices(checkBox = true),
            Dialog(tab = "Advanced", group = "Noise generation"));
          // Advanced dialog menu: Initialization
          parameter Boolean useGlobalSeed = true "= true: use global seed, otherwise ignore it" annotation(
            choices(checkBox = true),
            Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise));
          parameter Boolean useAutomaticLocalSeed = true "= true: use automatic local seed, otherwise use fixedLocalSeed" annotation(
            choices(checkBox = true),
            Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise));
          parameter Integer fixedLocalSeed = 1 "Local seed for the normal noise (any Integer number)" annotation(
            Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise and not useAutomaticLocalSeed));
          parameter Integer fixedLocalSeedRW = fixedLocalSeed + 1 "Local seed for the random walk noise(any Integer number)" annotation(
            Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise and not useAutomaticLocalSeed));
          parameter Real sigmaNoise = 1 "Standard Deviation of the white noise";
          parameter Real sigmaRW = 1 "Standard Deviation of the white noise generating the Random Walk";
          RocketControl.Components.Sensors.Internal.Noise.ClockedNormalNoise rwNoise(sigma = sigmaRW, useAutomaticLocalSeed = useAutomaticLocalSeed, useGlobalSeed = useGlobalSeed, enableNoise = enableNoise, fixedLocalSeed = fixedLocalSeedRW) annotation(
            Placement(visible = true, transformation(origin = {-30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Continuous.Integrator integrator annotation(
            Placement(visible = true, transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Math.Add add annotation(
            Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Sources.Constant const(k = 0) annotation(
            Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          RocketControl.Components.Sensors.Internal.Noise.ClockedNormalNoise normalNoise(sigma = sigmaNoise, useAutomaticLocalSeed = useAutomaticLocalSeed, useGlobalSeed = useGlobalSeed, enableNoise = enableNoise, fixedLocalSeed = fixedLocalSeed) annotation(
            Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        protected
          outer Modelica.Blocks.Noise.GlobalSeed globalSeed "Definition of global seed via inner/outer";
        equation
          connect(rwNoise.y, integrator.u) annotation(
            Line(points = {{-19, 50}, {-2, 50}}, color = {0, 0, 127}));
          connect(add.y, y) annotation(
            Line(points = {{72, 0}, {110, 0}}, color = {0, 0, 127}));
          connect(integrator.y, add.u1) annotation(
            Line(points = {{22, 50}, {40, 50}, {40, 6}, {48, 6}}, color = {0, 0, 127}));
          connect(const.y, rwNoise.u) annotation(
            Line(points = {{-59, 50}, {-42, 50}}, color = {0, 0, 127}));
          connect(u, normalNoise.u) annotation(
            Line(points = {{-120, 0}, {-42, 0}}, color = {0, 0, 127}));
          connect(normalNoise.y, add.u2) annotation(
            Line(points = {{-19, 0}, {40, 0}, {40, -6}, {48, -6}}, color = {0, 0, 127}));
          annotation(
            Icon(graphics = {Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-38, -33}, {-32, -39}}, endAngle = 360), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-81, 90}, {-89, 68}, {-73, 68}, {-81, 90}}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-70, 3}, {-64, -3}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-28, -15}, {-22, -21}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{30, 53}, {36, 47}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-84, -13}, {-78, -19}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{48, -47}, {54, -53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{40, 19}, {46, 13}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-46, 59}, {-40, 53}}, endAngle = 360), Line(points = {{-90, -23}, {82, -23}}, color = {192, 192, 192}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{20, -19}, {26, -25}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-54, -23}, {-48, -29}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-2, 53}, {4, 47}}, endAngle = 360), Line(points = {{-81, 78}, {-81, -90}}, color = {192, 192, 192}), Line(points = {{-35, 25}, {-35, -35}, {-25, -35}, {-25, -17}, {-15, -17}, {-15, -45}, {-5, -45}, {-5, 37}, {1, 37}, {1, 51}, {7, 51}, {7, -5}, {17, -5}, {17, 7}, {23, 7}, {23, -23}, {33, -23}, {33, 49}, {43, 49}, {43, 15}, {51, 15}, {51, -51}, {61, -51}}, color = {0, 0, 127}, pattern = LinePattern.Dot), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{14, 9}, {20, 3}}, endAngle = 360), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{91, -22}, {69, -14}, {69, -30}, {91, -22}}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-8, 39}, {-2, 33}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-62, -47}, {-56, -53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-18, -41}, {-12, -47}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{4, -1}, {10, -7}}, endAngle = 360), Line(points = {{-81, -17}, {-67, -17}, {-67, -1}, {-59, -1}, {-59, -49}, {-51, -49}, {-51, -27}, {-43, -27}, {-43, 57}, {-35, 57}, {-35, 25}}, color = {0, 0, 127}, pattern = LinePattern.Dot), Text(lineColor = {175, 175, 175}, extent = {{-66, 92}, {94, 66}}, textString = "sigmaRW=sigmaRW"), Text(lineColor = {175, 175, 175}, extent = {{-70, -68}, {94, -96}}, textString = "sigma=%sigma"), Line(origin = {5, 0.35}, points = {{-86.0123, -23.0123}, {-54.0123, -9.01233}, {-40.0123, -17.0123}, {-14.0123, 2.98767}, {9.98767, -5.01233}, {31.9877, 10.9877}, {45.9877, -1.01233}, {63.9877, 4.98767}}, color = {255, 0, 0})}));
        end ClockedSensorNoise;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Noise;

      model VectorHold
        parameter Integer n(min = 1) = 1 "Number of signals to hold" annotation(
          Evaluate = true);
        Modelica.Blocks.Interfaces.RealOutput y[n] annotation(
          Placement(visible = true, transformation(extent = {{100, -10}, {120, 10}}, rotation = 0), iconTransformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput u[n] annotation(
          Placement(visible = true, transformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0), iconTransformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0)));
      equation
        for i in 1:n loop
          y[i] = noEvent(hold(u[i]));
        end for;
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 130}, {150, 90}}, textString = "%name"), Line(points = {{-60, -40}, {-60, 0}, {-100, 0}}, color = {0, 0, 127}), Line(points = {{-60, -40}, {-20, -40}, {-20, 20}, {20, 20}, {20, 60}, {60, 60}, {60, 0}, {100, 0}, {100, 0}, {100, 0}, {100, 0}, {120, 0}}, color = {0, 0, 127}), Text(origin = {0, -71}, lineColor = {86, 86, 86}, extent = {{-60, 21}, {60, -21}}, textString = "n = %n")}));
      end VectorHold;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Internal;
  end Sensors;

  package Propulsion
    model M2000R
      parameter Modelica.Units.SI.Time start_delay = 0;
      Modelica.Blocks.Sources.TimeTable thrustCurve(offset = 0, shiftTime = start_delay, startTime = start_delay, table = [0, 0; 0.1, 1500; 0.2, 2250; 2.1, 2400; 4, 1750; 4.6, 100; 4.7, 0; 500, 0], timeScale = 1) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Internal.SolidMotorAndTank m2000r(Dext = 0.098, Din_0 = 0.03, Isp = 176, h = 0.732, m_0 = 5.368) annotation(
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

    model Sustainer
      parameter Modelica.Units.SI.Time start_delay = 0;
      Modelica.Blocks.Sources.TimeTable thrustCurve(offset = 0, shiftTime = start_delay, startTime = start_delay, table = [0, 0; 0.1, 500; 0.2, 1000; 2, 1000; 2.2, 402; 20, 400; 20.2, 0; 500, 0], timeScale = 1) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Internal.SolidMotorAndTank m2000r(Dext = 0.098, Din_0 = 0.03, Isp = 176, h = 0.732, m_0 = 5.368) annotation(
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

    model Cesaroni
      parameter Modelica.Units.SI.Time start_delay = 0;
      Modelica.Blocks.Sources.TimeTable thrustCurve(offset = 0, shiftTime = start_delay, startTime = start_delay, table = [0, 0; 0.016, 1421.724; 0.034, 1345.218; 0.049, 1502.479; 0.081, 1415.348; 0.21, 1432.349; 0.453, 1432.349; 0.809, 1462.102; 1.07, 1534.357; 1.28, 1540.732; 2.661, 1283.589; 2.843, 1277.214; 2.932, 1115.702; 3.037, 488.784; 3.163, 82.881; 3.284, 0.0; 500, 0], timeScale = 1) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Internal.SolidMotorAndTank l1350cs(Dext = 0.075, Din_0 = 0.02, Isp = 228.2, h = 0.532, m_0 = 1.905) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
    equation
      connect(l1350cs.frame_b, frame_b) annotation(
        Line(points = {{0, 10}, {0, 98}}));
      connect(thrustCurve.y, l1350cs.thrust) annotation(
        Line(points = {{-59, 0}, {-10, 0}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Text(origin = {2, -184}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Rectangle(origin = {0.51, 47.73}, fillColor = {85, 85, 85}, fillPattern = FillPattern.VerticalCylinder, extent = {{-59.94, 48.27}, {59.94, -48.27}}), Polygon(origin = {1, -30}, fillColor = {43, 88, 85}, fillPattern = FillPattern.VerticalCylinder, points = {{-21, 30}, {-51, 10}, {-71, -30}, {71, -30}, {49, 10}, {19, 30}, {5, 30}, {-21, 30}}), Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "M2000R")}));
    end Cesaroni;

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

      model SolidMotorAndTank
        parameter SI.Time Isp = 180 "Specific impulse in seconds";
        parameter Modelica.Units.SI.Length Dext "External diameter";
        parameter Modelica.Units.SI.Length Din_0 "Initial internal diameter";
        parameter Modelica.Units.SI.Length h "Height";
        parameter Modelica.Units.SI.Mass m_0 "Initial mass";
        RocketControl.Components.Propulsion.Internal.Inertia.SolidPropellantInertia solidPropellantInertia(Dext = Dext, Din_0 = Din_0, h = h, m_0 = m_0) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.Parts.BodyVariableMass bodyVariableMass(r_CM = {h / 2, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
          Placement(visible = true, transformation(origin = {40, 4}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealInput thrust annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        RocketControl.Components.Propulsion.Internal.GenericMotor genericMotor(Isp = Isp) annotation(
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
          extends PropellantInertiaBase;
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
    end Internal;
  end Propulsion;

  package Actuators
    block ServoMotor
  parameter Integer nservos(min = 1) = 4;
      parameter Real b[:] = {1} "Numerator coefficients of transfer function (e.g., 2*s+3 is specified as {2,3})";
      parameter Real a[:] = {1} "Denominator coefficients of transfer function (e.g., 5*s+6 is specified as {5,6})";
      parameter SI.Angle saturation_angle = from_deg(10);
      Modelica.Blocks.Interfaces.RealInput setpoint[nservos] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput servo_pos[nservos] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.TransferFunction servoResponse[nservos](each a = a, each b = b, each initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
        Placement(visible = true, transformation(origin = {-56, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Nonlinear.Limiter servoSaturation[nservos](each uMax = saturation_angle)  annotation(
        Placement(visible = true, transformation(origin = {32, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(setpoint, servoResponse.u) annotation(
        Line(points = {{-120, 0}, {-68, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(servoResponse.y, servoSaturation.u) annotation(
        Line(points = {{-44, 0}, {20, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(servoSaturation.y, servo_pos) annotation(
        Line(points = {{44, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
      annotation(
        Icon(graphics = {Rectangle(fillColor = {238, 238, 238}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Polygon(origin = {45, 0}, fillColor = {93, 255, 147}, fillPattern = FillPattern.Forward, points = {{-35, 62}, {-35, -62}, {35, -36}, {35, 20}, {-35, 62}}), Polygon(origin = {-28, 0}, fillColor = {203, 203, 203}, fillPattern = FillPattern.HorizontalCylinder, points = {{38, 8}, {0, 8}, {0, 40}, {-38, 40}, {-38, -40}, {0, -40}, {0, -8}, {38, -8}, {38, 8}}), Line(origin = {-7.28, -1.55}, points = {{8.7929, 18}, {4.79289, 30}, {-5.20711, 30}, {-11.2071, 18}, {-11.2071, -4}, {-11.2071, -18}, {-5.20711, -30}, {4.79289, -30}, {8.79289, -18}, {8.79289, -18}}, color = {255, 0, 0}, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 10), Text(origin = {0, -125}, lineColor = {0, 0, 255}, extent = {{-140, 25}, {140, -25}}, textString = "%name")}));
    end ServoMotor;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Actuators;

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
      Components.Parts.Forces.LinearSpringDamperParallel normalRailSpring1(c = c_norm / 2, d = d_norm / 2, n = n_norm, s_0 = s_0_norm, s_start = s_start_norm, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Components.Parts.Forces.LinearSpringDamperParallel normalRailSpring2(c = c_norm / 2, d = d_norm / 2, n = n_norm, s_0 = s_0_norm, s_fixed = s_fixed_norm2, s_start = s_start_norm, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Components.Parts.Forces.LinearSpringDamperParallel axialRailSpring(c = c_ax, d = d_ax, n = n_ax, s_0 = s_0_ax, s_start = s_start_ax, s_fixed = s_fixed_ax, useEnablePort = true) annotation(
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
      parameter SI.Angle azimuth;
      parameter SI.Angle elevation;
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
      LaunchRailPresenceSensor launchPadSensorBase(n = {1, 0, 0}, rail_length = 1e-8) annotation(
        Placement(visible = true, transformation(origin = {10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      LaunchLug lug_aft(c_ax = c_z / 2, c_norm = c_y / 2, d_ax = d_z / 2, d_norm = d_y / 2, lug_length = lug_length, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      LaunchLug lug_bow(c_ax = c_z / 2, c_norm = c_y / 2, d_ax = d_z / 2, d_norm = d_y / 2, lug_length = lug_length, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {-10, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.Parts.Forces.LinearSpringDamperParallel linearSpringDamperParallel(c = c_x, d = d_x, n = {1, 0, 0}, useEnablePort = true) annotation(
        Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      LaunchRailPresenceSensor launchPadSensorAft(n = {1, 0, 0}, rail_length = rail_length) annotation(
        Placement(visible = true, transformation(origin = {-70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedRotation launchRailAngle(angles = to_deg({azimuth, elevation, 0}), rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence, sequence = {3, 2, 1}) annotation(
        Placement(visible = true, transformation(origin = {-62, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      LaunchRailPresenceSensor launchPadSensorBow(n = {1, 0, 0}, rail_length = rail_length) annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
        Placement(visible = true, transformation(origin = {-100, -50}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b_lug_bow annotation(
        Placement(visible = true, transformation(origin = {100, 80}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b_lug_aft annotation(
        Placement(visible = true, transformation(origin = {100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Blocks.Logical.Not not1 annotation(
        Placement(visible = true, transformation(origin = {50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanOutput liftoff annotation(
        Placement(visible = true, transformation(origin = {108, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, -104}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    equation
      connect(frame_a, launchRailAngle.frame_a) annotation(
        Line(points = {{-100, -50}, {-72, -50}}));
      connect(freeMotionScalarInit.frame_a, launchRailAngle.frame_b) annotation(
        Line(points = {{20, -10}, {-40, -10}, {-40, -50}, {-52, -50}}, color = {95, 95, 95}));
      connect(launchRailAngle.frame_b, launchPadSensorAft.frame_rail) annotation(
        Line(points = {{-52, -50}, {-40, -50}, {-40, -20}, {-70, -20}, {-70, 0}}));
      connect(launchRailAngle.frame_b, linearSpringDamperParallel.frame_a) annotation(
        Line(points = {{-52, -50}, {-40, -50}, {-40, -90}, {70, -90}, {70, -80}}, color = {95, 95, 95}));
      connect(launchPadSensorBase.frame_rail, launchRailAngle.frame_b) annotation(
        Line(points = {{10, -80}, {10, -90}, {-40, -90}, {-40, -50}, {-52, -50}}));
      connect(linearSpringDamperParallel.frame_b, frame_b_lug_aft) annotation(
        Line(points = {{70, -60}, {70, 20}, {100, 20}}));
      connect(freeMotionScalarInit.frame_b, frame_b_lug_aft) annotation(
        Line(points = {{40, -10}, {60, -10}, {60, 20}, {100, 20}}));
      connect(lug_bow.frame_a, launchRailAngle.frame_b) annotation(
        Line(points = {{-20, 68}, {-40, 68}, {-40, -50}, {-52, -50}}, color = {95, 95, 95}));
      connect(lug_bow.frame_b, frame_b_lug_bow) annotation(
        Line(points = {{0, 68}, {50, 68}, {50, 80}, {100, 80}}));
      connect(launchPadSensorBow.frame_rail, launchRailAngle.frame_b) annotation(
        Line(points = {{-70, 60}, {-70, 30}, {-40, 30}, {-40, -50}, {-52, -50}}, color = {95, 95, 95}));
      connect(launchPadSensorBow.frame_lug, lug_bow.frame_b) annotation(
        Line(points = {{-60, 66}, {-31, 66}, {-31, 68}, {0, 68}}));
      connect(launchPadSensorBow.y, lug_bow.enable) annotation(
        Line(points = {{-60, 77}, {-44, 77}, {-44, 84}, {-10, 84}, {-10, 78}}, color = {255, 0, 255}));
      connect(launchPadSensorAft.frame_lug, lug_aft.frame_b) annotation(
        Line(points = {{-60, 6}, {14, 6}, {14, 20}, {0, 20}}, color = {95, 95, 95}));
      connect(lug_aft.frame_b, frame_b_lug_aft) annotation(
        Line(points = {{0, 20}, {100, 20}}, color = {95, 95, 95}));
      connect(launchPadSensorBase.y, linearSpringDamperParallel.enable) annotation(
        Line(points = {{20, -62}, {34, -62}, {34, -70}, {61, -70}}, color = {255, 0, 255}));
      connect(launchPadSensorBase.frame_lug, linearSpringDamperParallel.frame_b) annotation(
        Line(points = {{20, -74}, {38, -74}, {38, -54}, {70, -54}, {70, -60}}));
      connect(launchPadSensorAft.y, lug_aft.enable) annotation(
        Line(points = {{-60, 17}, {-32, 17}, {-32, 36}, {-10, 36}, {-10, 30}}, color = {255, 0, 255}));
      connect(lug_aft.frame_a, launchRailAngle.frame_b) annotation(
        Line(points = {{-20, 20}, {-40, 20}, {-40, -50}, {-52, -50}}));
      connect(not1.u, launchPadSensorBase.y) annotation(
        Line(points = {{38, -40}, {20, -40}, {20, -62}}, color = {255, 0, 255}));
      connect(not1.y, liftoff) annotation(
        Line(points = {{62, -40}, {108, -40}}, color = {255, 0, 255}));
      annotation(
        Icon(graphics = {Polygon(origin = {-20, -6}, fillColor = {85, 122, 162}, fillPattern = FillPattern.VerticalCylinder, points = {{-28, -42}, {-14, -50}, {-14, -42}, {26, 38}, {28, 50}, {20, 42}, {-20, -38}, {-28, -42}}), Polygon(origin = {-9, 8}, fillColor = {222, 222, 222}, fillPattern = FillPattern.Forward, points = {{17, 88}, {-71, -78}, {-71, -88}, {71, -88}, {49, -80}, {-55, -80}, {17, 62}, {17, 88}}), Rectangle(origin = {6, 20}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Rectangle(origin = {-22, -36}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Text(lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name"), Text(origin = {-760.66, 76}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "a"), Text(origin = {-566.66, 74}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "bow"), Text(origin = {-227.774, 14}, lineColor = {128, 128, 128}, extent = {{277.774, -29}, {377.774, -58}}, textString = "aft"), Line(origin = {40, -48}, points = {{-62, 12}, {20, 12}, {20, -12}, {62, -12}}), Line(origin = {53, 40}, points = {{-47, -20}, {7, -20}, {7, 20}, {47, 20}})}));
    end LaunchRail;
  
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
  end LaunchPad;

  package Parts

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
        Components.Parts.Forces.LinearSpring linearSpring(c = c, n = n, s(fixed = s_fixed, start = s_start), s_0 = s_0, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.Parts.Forces.LinearDamper linearDamper(d = d, n = n, s(start = s_start), useEnablePort = true) annotation(
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
        parameter SI.TranslationalSpringConstant c(final min = 0) "Spring constant";
        parameter SI.Length s_unstretched = 0 "Unstretched spring length";
        parameter SI.TranslationalDampingConstant d(final min = 0) = 0 "Damping constant";
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
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {0, -0.5, sqrt(3) / 2} * r) annotation(
          Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = c, d = d, s_unstretched = s_unstretched) annotation(
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

      model CompressionSpring "Spring that reacts only when compressed"
        extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
        import Modelica.Mechanics.MultiBody.Frames.resolve2;
        import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
        parameter Real n[3] = {1, 0, 0} "Direction along which the spring is acting";
        parameter Modelica.Units.SI.ModulusOfElasticity c;
        SI.Position s(fixed = false, start = 0);
        SI.Position r_rel[3];
        SI.Force f[3];
      equation
        r_rel = resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
        s = r_rel * n;
        frame_b.t = zeros(3);
        frame_a.t = zeros(3);
        if s < 0 then
          f = -c * r_rel;
        else
          f = zeros(3);
        end if;
        frame_a.f = f;
        frame_b.f = resolveRelative(-f, frame_a.R, frame_b.R);
        annotation(
          Icon(graphics = {Line(points = {{-100, 0}, {-58, 0}, {-43, -30}, {-13, 30}, {17, -30}, {47, 30}, {62, 0}, {100, 0}}), Line(origin = {-23.8859, 59.9583}, points = {{44, 0}, {4, 0}}, thickness = 0.75), Text(origin = {0, 22}, extent = {{-150, -75}, {150, -45}}, textString = "c=%c"), Text(origin = {0, -12}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Text(origin = {4, -176}, lineColor = {0, 0, 255}, extent = {{-150, 85}, {150, 45}}, textString = "%name"), Polygon(origin = {30, 60}, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}}), Polygon(origin = {-30, 60}, rotation = 180, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}})}));
      end CompressionSpring;

      model CompressionDamper "Damper that acts only on a specific direction and can be enabled / disabled"
        extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
        import Modelica.Mechanics.MultiBody.Frames.resolve2;
        import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
        parameter Real n[3] = {1, 0, 0} "Direction along which the damper is acting";
        parameter Modelica.Units.SI.DampingCoefficient d;
        SI.Position s(fixed = false, start = 0);
        SI.Position r_rel[3];
        SI.Force f[3];
      equation
        r_rel = resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
        s = r_rel * n;
        frame_b.t = zeros(3);
        frame_a.t = zeros(3);
        if der(s) < 0 and s < 0 then
          f = -d * der(r_rel);
        else
          f = zeros(3);
        end if;
        frame_a.f = f;
        frame_b.f = resolveRelative(-f, frame_a.R, frame_b.R);
        annotation(
          Icon(graphics = {Text(origin = {4, -176}, lineColor = {0, 0, 255}, extent = {{-150, 85}, {150, 45}}, textString = "%name"), Text(origin = {0, -12}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Rectangle(origin = {8, 0}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {30, -30}}), Line(origin = {7.40383, -1.76063e-06}, points = {{-101, 0}, {-60, 0}}), Line(origin = {7.40383, -1.76063e-06}, points = {{30, 0}, {100, 0}}), Line(origin = {7.40383, -1.76063e-06}, points = {{-60, -30}, {-60, 30}}), Text(origin = {0, 18}, extent = {{-150, -75}, {150, -45}}, textString = "d=%d"), Line(visible = false, origin = {7.40383, -1.76063e-06}, points = {{-100, -99}, {-100, -25}, {-10, -25}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Line(origin = {7.40383, -1.76063e-06}, points = {{-60, -30}, {60, -30}}), Line(origin = {7.40383, -1.76063e-06}, points = {{-60, 30}, {60, 30}}), Polygon(origin = {-30, 60}, rotation = 180, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}}), Line(origin = {-23.8859, 59.9583}, points = {{44, 0}, {4, 0}}, thickness = 0.75), Polygon(origin = {30, 60}, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}})}));
      end CompressionDamper;

      model CompressionSpringDamper "Spring and damper in parallel that act only on a specific direction and can be enabled / disabled"
        extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
        parameter Real n[3] = {1, 0, 0} "Direction along which the spring is acting";
        parameter SI.ModulusOfElasticity c;
        parameter SI.DampingCoefficient d;
        Components.Forces.CompressionSpring linearSpring(c = c, n = n) annotation(
          Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.Forces.CompressionDamper linearDamper(d = d, n = n) annotation(
          Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(linearSpring.frame_b, frame_b) annotation(
          Line(points = {{10, 40}, {60, 40}, {60, 0}, {100, 0}}));
        connect(linearDamper.frame_b, frame_b) annotation(
          Line(points = {{10, -40}, {60, -40}, {60, 0}, {100, 0}}, color = {95, 95, 95}));
        connect(frame_a, linearDamper.frame_a) annotation(
          Line(points = {{-100, 0}, {-60, 0}, {-60, -40}, {-10, -40}}));
        connect(linearSpring.frame_a, frame_a) annotation(
          Line(points = {{-10, 40}, {-60, 40}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
        annotation(
          Icon(graphics = {Line(points = {{-52, -40}, {68, -40}}), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}, endAngle = 360), Text(origin = {-82, 122}, extent = {{-150, -75}, {150, -45}}, textString = "d=%d"), Line(points = {{38, -70}, {80, -70}}), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}, endAngle = 360), Line(points = {{80, 40}, {80, -70}}), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Text(origin = {-82, 178}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Text(visible = false, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Text(visible = false, origin = {0, -28}, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Rectangle(fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-52, -40}, {38, -100}}), Line(points = {{-80, 40}, {-60, 40}, {-45, 10}, {-15, 70}, {15, 10}, {45, 70}, {60, 40}, {80, 40}}), Text(origin = {-82, 154}, extent = {{-150, -75}, {150, -45}}, textString = "c=%c"), Line(points = {{-80, -70}, {-52, -70}}), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}, endAngle = 360), Line(points = {{80, 0}, {100, 0}}), Line(points = {{-52, -100}, {68, -100}}), Line(visible = false, points = {{-100, -101}, {-100, -80}, {-6, -80}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}, endAngle = 360), Line(origin = {-80, -15}, points = {{0, 55}, {0, -55}}), Line(origin = {-89, 0}, points = {{9, 0}, {-7, 0}, {-9, 0}}), Line(origin = {-24.6905, -11.2486}, points = {{44, 0}, {4, 0}}, thickness = 0.75), Polygon(origin = {-30, -12}, rotation = 180, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}}), Polygon(origin = {30, -12}, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}})}));
      end CompressionSpringDamper;
    end Forces;
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
      Interfaces.MassPropertiesInput massInput annotation(
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

    model LandDetector
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
      parameter SI.Position groundlevel = 0;
    equation
      frame_a.f = zeros(3);
      frame_a.t = zeros(3);
      when frame_a.r_0[3] > groundlevel and time > 10 then
        terminate("Simulation terminated successfully");
      end when;
      annotation(
        Icon(graphics = {Polygon(origin = {-3, -53}, lineColor = {0, 170, 127}, fillColor = {85, 255, 127}, fillPattern = FillPattern.HorizontalCylinder, points = {{-53, 41}, {-73, 31}, {-83, -19}, {-73, -39}, {67, -39}, {87, -27}, {97, 21}, {87, 41}, {-53, 41}}), Ellipse(origin = {6, -51}, fillColor = {0, 134, 98}, fillPattern = FillPattern.Solid, extent = {{38, 13}, {-38, -13}}, endAngle = 360), Polygon(origin = {10, -10}, fillColor = {0, 170, 255}, fillPattern = FillPattern.Solid, points = {{-6, -42}, {18, -34}, {6, -24}, {-16, 38}, {-24, 48}, {-24, 34}, {-4, -28}, {-6, -42}}), Polygon(origin = {38, 61}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, points = {{-32, 25}, {20, -35}, {28, -25}, {32, -7}, {28, 13}, {18, 29}, {0, 35}, {-20, 33}, {-32, 25}}), Line(origin = {6.18874, 57.3515}, points = {{-0.188744, 28.6485}, {-12.1887, -29.3515}, {11.8113, 14.6485}, {11.8113, 14.6485}}), Line(origin = {18.3124, 43.2299}, points = {{11.6876, 14.7701}, {-24.3124, -15.2299}, {23.6876, 0.770139}}), Line(origin = {26.4447, 31.0503}, points = {{21.5553, 6.94969}, {-32.4447, -3.05031}, {31.5553, -5.05031}, {31.5553, -5.05031}}), Text(origin = {6, -204}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Text(origin = {-52, 74}, extent = {{-46, 28}, {46, -28}}, textString = "z > 0")}));
    end LandDetector;

    model DescentDetector
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
      parameter SI.Velocity descent_rate = 10;
    equation
      frame_a.f = zeros(3);
      frame_a.t = zeros(3);
      when der(frame_a.r_0[3]) > descent_rate and time > 10 then
        terminate("Simulation terminated successfully");
      end when;
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 170, 255}, fillColor = {170, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, 100}, {100, -100}}, radius = 20), Polygon(origin = {10, -10}, fillColor = {0, 170, 255}, fillPattern = FillPattern.Solid, points = {{-6, -42}, {18, -34}, {6, -24}, {-16, 38}, {-24, 48}, {-24, 34}, {-4, -28}, {-6, -42}}), Polygon(origin = {38, 61}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, points = {{-32, 25}, {20, -35}, {28, -25}, {32, -7}, {28, 13}, {18, 29}, {0, 35}, {-20, 33}, {-32, 25}}), Line(origin = {6.18874, 57.3515}, points = {{-0.188744, 28.6485}, {-12.1887, -29.3515}, {11.8113, 14.6485}, {11.8113, 14.6485}}), Line(origin = {18.3124, 43.2299}, points = {{11.6876, 14.7701}, {-24.3124, -15.2299}, {23.6876, 0.770139}}), Line(origin = {26.4447, 31.0503}, points = {{21.5553, 6.94969}, {-32.4447, -3.05031}, {31.5553, -5.05031}, {31.5553, -5.05031}}), Text(origin = {6, -204}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Text(origin = {9, 131}, fillColor = {170, 255, 255}, extent = {{-131, 31}, {131, -31}}, textString = "vz > %descent_rate"), Line(origin = {39, -74}, points = {{-17, 20}, {17, -20}}, color = {255, 0, 0}, thickness = 0.5, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15)}));
    end DescentDetector;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Parts;

  package Blocks

    model Track
      extends Icon;
      Modelica.Blocks.Interfaces.RealInput v[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput track(final quantity = "Angle", final unit = "rad", displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Real track_signed = 0;
    equation
      if noEvent(abs(v[1]) > 1e-3) then
        track = atan2(v[2], v[1]);
      else
        track = sign(v[2]) * pi / 2;
      end if;
      annotation(
        Icon(graphics = {Line(origin = {-15.86, -16.41}, points = {{-44, 85}, {-46, -45}, {76, -45}}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 10), Line(origin = {13.72, 25.15}, points = {{-73, 44}, {47, 44}, {47, -86}}, pattern = LinePattern.Dash), Line(origin = {-15.86, -16.41}, points = {{-46, -45}, {76, 85}}, color = {170, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {-34.104, 15.4601}, points = {{-25.1708, 30.1708}, {-15.1708, 30.1708}, {-3.17082, 26.1708}, {4.82918, 22.1708}, {12.8292, 16.1708}, {22.8292, 6.17082}, {34.8292, -9.82918}}, color = {255, 0, 0}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Ellipse(origin = {-60, -60}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Text(origin = {-130, -32}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "v"), Text(origin = {120, -26}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "track")}));
    end Track;

    model GlideAngle
      extends Icon;
      parameter SI.Velocity v_small = 1e-5;
      Modelica.Blocks.Interfaces.RealInput v[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput angle(displayUnit = "deg", quantity = "Angle", unit = "rad") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Velocity vnorm;
    equation
      vnorm = norm(v);
      if noEvent(vnorm > v_small) then
        angle = -asin(v[3] / vnorm);
      else
        angle = 0;
      end if;
      annotation(
        Icon(graphics = {Ellipse(origin = {-60, 68}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Line(origin = {-15.86, -16.41}, points = {{76, 85}, {-44, 85}, {-44, -45}}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 10), Line(origin = {15.36, 4.44}, points = {{45, 64}, {45, -66}, {-75, -66}}, pattern = LinePattern.Dash), Line(origin = {91.73, -9.79}, points = {{-152, 79}, {-32, -51}}, color = {170, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {10.1974, 19.1998}, points = {{0.8292, 48.1708}, {2.8292, 40.1708}, {2.82918, 30.1708}, {0.82918, 20.1708}, {-3.1708, 10.1708}, {-11.1708, 0.17082}, {-17.1708, -5.82918}}, color = {255, 0, 0}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Text(origin = {120, -26}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "glideSlope"), Text(origin = {-130, -32}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "v")}));
    end GlideAngle;

    model Downrange
      extends Icon;
      Modelica.Blocks.Interfaces.RealInput x[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput d_downrange annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      d_downrange = sqrt(x[1] ^ 2 + x[2] ^ 2);
      annotation(
        Icon(graphics = {Ellipse(origin = {58, 56}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Line(origin = {-2.48, -6.47}, points = {{-46, -45}, {50, 53}}, color = {170, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {-2.86, -6.47}, points = {{-46, 17}, {-46, -45}, {14, -45}}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 10), Text(origin = {-130, -40}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "pos")}));
    end Downrange;

    block HeadingRate
      extends Icon;
      Modelica.Blocks.Interfaces.RealInput w[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput q[4] annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput heading_rate(final unit = "rad/s", final quantity = "AngularVelocity", displayUnit = "deg/s") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.AngularVelocity w_yz[3];
      SI.AngularVelocity w_w[3];
    equation
      w_yz = cat(1, {0}, w[2:3]);
      w_w = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve1(q, w_yz);
      heading_rate = w_w[3];
      annotation(
        Icon(graphics = {Line(origin = {11.7976, 8.19235}, points = {{-25.1708, 30.1708}, {-15.1708, 30.1708}, {-3.17082, 26.1708}, {4.82918, 22.1708}, {12.8292, 16.1708}, {22.8292, 6.17082}, {32.8292, -17.8292}}, color = {255, 0, 0}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Ellipse(origin = {-40, -60}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Line(origin = {-15.86, -16.41}, points = {{-16, -33}, {76, 85}}, color = {170, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Text(origin = {-79, 60}, lineColor = {120, 120, 120}, extent = {{-21, 20}, {21, -20}}, textString = "w"), Text(origin = {-79, -60}, lineColor = {120, 120, 120}, extent = {{-21, 20}, {21, -20}}, textString = "q")}));
    end HeadingRate;

    block EulerRates
      extends Icon;
      Modelica.Blocks.Interfaces.RealInput w[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput q[4] annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput euler_rates[3](each displayUnit = "deg/s", each final quantity = "AngularVelocity", each final unit = "rad/s") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
      Real M[3, 3];
      Real a[3];
    equation
      a = RocketControl.Math.quat2euler(q);
      if noEvent(abs(abs(a[2]) - Modelica.Constants.pi / 2) < 1e-3) then
        M = [0, sin(a[3]) / cos(1e-3 * sign(a[2])), cos(a[3]) / cos(1e-3 * sign(a[2])); 0, cos(a[3]), -sin(a[3]); 1, sin(a[3]) * tan(1e-3 * sign(a[2])), cos(a[3]) * tan(1e-3 * sign(a[2]))];
      else
        M = [0, sin(a[3]) / cos(a[2]), cos(a[3]) / cos(a[2]); 0, cos(a[3]), -sin(a[3]); 1, sin(a[3]) * tan(a[2]), cos(a[3]) * tan(a[2])];
      end if;
      euler_rates = M * w;
      annotation(
        Icon(graphics = {Line(origin = {9, 15.34}, points = {{-46, 29}, {-46, -45}, {20, -75}}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 10), Line(origin = {-2.8, 6.6}, points = {{-34, -35}, {36, 17}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Ellipse(origin = {-36, -28}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Line(origin = {33.22, 23.49}, points = {{-29.1708, 16.1708}, {-17.1708, 20.1708}, {-3.17082, 20.1708}, {8.82918, 16.1708}, {16.8292, 8.1708}, {22.8292, -3.82918}, {22.8292, -27.8292}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Line(origin = {22.56, -59.08}, rotation = -90, points = {{-29.1708, 16.1708}, {-17.1708, 20.1708}, {-3.17082, 20.1708}, {8.82918, 16.1708}, {16.8292, 8.1708}, {22.8292, -3.82918}, {22.8292, -27.8292}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Line(origin = {-35.14, 35.84}, rotation = -90, points = {{-29.1708, 16.1708}, {-17.1708, 20.1708}, {-3.17082, 20.1708}, {8.82918, 16.1708}, {16.8292, 8.1708}, {22.8292, -3.82918}, {22.8292, -27.8292}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Text(origin = {-74, 61}, lineColor = {102, 102, 102}, extent = {{-26, 19}, {26, -19}}, textString = "w"), Text(origin = {-74, -59}, lineColor = {102, 102, 102}, extent = {{-26, 19}, {26, -19}}, textString = "q")}));
    end EulerRates;

    model Icon
    equation

      annotation(
        Icon(graphics = {Rectangle(lineColor = {255, 0, 0}, fillColor = {170, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-100, 100}, {100, -100}}), Rectangle(lineColor = {255, 0, 0}, fillColor = {170, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {4, -129}, lineColor = {0, 0, 255}, extent = {{-180, 29}, {180, -29}}, textString = "%name")}));
    end Icon;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Blocks;
end Components;
