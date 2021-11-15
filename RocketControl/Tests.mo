within RocketControl;

package Tests

  model MotionFrame
    import Modelica.Units.SI;
    import Modelica.Mechanics.MultiBody.Frames;
    parameter SI.Velocity p0[3] = {0, 0, 0};
    parameter SI.Velocity v0[3];
    parameter SI.Angle angles0[3](each displayUnit = "deg") = {0, 0, 0};
    final parameter Frames.Orientation R0 = Modelica.Mechanics.MultiBody.Frames.axesRotations({3, 2, 1}, angles0, {0, 0, 0});
    parameter SI.AngularVelocity w[3](each displayUnit = "deg/s") = {0, 0, 0};
    SI.Angle angles[3];
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(visible = true, transformation(origin = {98, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {98, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  initial equation
    frame_b.r_0 = p0;
    angles = {0, 0, 0};
  equation
    Connections.root(frame_b.R);
    der(frame_b.r_0) = v0;
    der(angles) = w;
    frame_b.R = Frames.absoluteRotation(Frames.axesRotations({1, 2, 3}, angles, der(angles)), R0);
    annotation(
      Icon(graphics = {Ellipse(origin = {0.04, 1.95}, fillColor = {255, 255, 255}, fillPattern = FillPattern.CrossDiag, extent = {{-99.96, 99.95}, {99.96, -99.95}}, endAngle = 360)}));
  end MotionFrame;

  package Components
    package Sensors
      model RealGyroscopeTest
        RocketControl.Components.Sensors.RealGyroscope biasOnly(bias = {0.1, -0.5, 0.6}, noiseSamplePeriod = 0.01001, samplingRate = 100, sigmaARW = 0, sigmaRRW = 0) annotation(
          Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(
          Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealGyroscope noiseOnly(bias = {0, 0, 0}, noiseSamplePeriod = 0.01001, samplingRate = 100, sigmaARW = 0.1, sigmaRRW = 0.01) annotation(
          Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealGyroscope allErrors(bias = {0.1, -0.5, 0.6}, noiseSamplePeriod = 0.01001, samplingRate = 100, sigmaARW = 0.1, sigmaRRW = 0.01) annotation(
          Placement(visible = true, transformation(origin = {-10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Tests.MotionFrame motionFrame(v0 = {0, 0, 0}, w(displayUnit = "rad/s") = {5, 10, -5}) annotation(
          Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(biasOnly.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-20, 50}, {-40, 50}, {-40, 0}, {-60, 0}}, color = {95, 95, 95}));
        connect(noiseOnly.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-20, 0}, {-60, 0}}));
        connect(allErrors.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-20, -50}, {-40, -50}, {-40, 0}, {-60, 0}}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end RealGyroscopeTest;

      model RealAccelerometerTest
        inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(
          Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealAccelerometer idealAccelerometer annotation(
          Placement(visible = true, transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealAccelerometer realAccelerometer(acc_max = 10, bias = {0, 0, 0}, bits = 16, fixedLocalSeed = {5, 20, 4231}, samplePeriodMs = 10, sigmaBiasInstability = 1, sigmaNoise = 1) annotation(
          Placement(visible = true, transformation(origin = {50, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
          Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(fixed.frame_b, realAccelerometer.frame_a) annotation(
          Line(points = {{-50, 0}, {4, 0}, {4, -34}, {40, -34}}, color = {95, 95, 95}));
        connect(fixed.frame_b, idealAccelerometer.frame_a) annotation(
          Line(points = {{-50, 0}, {4, 0}, {4, 30}, {40, 30}}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end RealAccelerometerTest;

      model Samplers
        Modelica.Blocks.Sources.Sine sine(amplitude = 10, f = 0.1) annotation(
          Placement(visible = true, transformation(origin = {-82, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.SampleClockedWithADeffects sample(bias = 2, biased = true, noise(sigma = 1), noisy = true) annotation(
          Placement(visible = true, transformation(origin = {0, 20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = 10) annotation(
          Placement(visible = true, transformation(origin = {-60, -80}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.SampleClockedWithADeffects sampleClockedWithADeffects(bias = 2, biased = true, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = 1, sigmaRW = 10, useAutomaticLocalSeed = false, fixedLocalSeed = 2088765250, fixedLocalSeedRW = 253619768), noisy = true) annotation(
          Placement(visible = true, transformation(origin = {0, -44}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.SampleClockedWithADeffects sampleClockedWithADeffects2(redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = 1, sigmaRW = 10, useAutomaticLocalSeed = false, fixedLocalSeed = 839930499, fixedLocalSeedRW = 542713403), bias = 2, biased = true, limited = true, noisy = true, quantized = true, yMax = 12) annotation(
          Placement(visible = true, transformation(origin = {0, -20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Components.Sensors.Internal.SampleClockedWithADeffects sampleClockedWithADeffects1(bias = 2, biased = true, noise(sigma = 1), noisy = true) annotation(
          Placement(visible = true, transformation(origin = {0, 40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      equation
        connect(sine.y, sample.u) annotation(
          Line(points = {{-71, 0}, {-40, 0}, {-40, 20}, {-8, 20}}, color = {0, 0, 127}));
        connect(periodicClock1.y, sample.clock) annotation(
          Line(points = {{-54, -80}, {0, -80}, {0, 12}}, color = {175, 175, 175}));
        connect(sine.y, sampleClockedWithADeffects.u) annotation(
          Line(points = {{-70, 0}, {-40, 0}, {-40, -44}, {-7, -44}}, color = {0, 0, 127}));
        connect(periodicClock1.y, sampleClockedWithADeffects.clock) annotation(
          Line(points = {{0, -51}, {0, -80}, {-54, -80}}, color = {175, 175, 175}));
        connect(sine.y, sampleClockedWithADeffects.u) annotation(
          Line(points = {{-70, 0}, {-40, 0}, {-40, -40}, {-8, -40}}, color = {0, 0, 127}));
        connect(sampleClockedWithADeffects.clock, periodicClock1.y) annotation(
          Line(points = {{0, -48}, {0, -80}, {-54, -80}}, color = {175, 175, 175}));
        connect(sine.y, sampleClockedWithADeffects2.u) annotation(
          Line(points = {{-70, 0}, {-40, 0}, {-40, -20}, {-8, -20}}, color = {0, 0, 127}));
        connect(sampleClockedWithADeffects2.clock, periodicClock1.y) annotation(
          Line(points = {{0, -28}, {0, -80}, {-54, -80}}, color = {175, 175, 175}));
        connect(sine.y, sampleClockedWithADeffects1.u) annotation(
          Line(points = {{-70, 0}, {-40, 0}, {-40, 40}, {-8, 40}}, color = {0, 0, 127}));
        connect(periodicClock1.y, sampleClockedWithADeffects1.clock) annotation(
          Line(points = {{-54, -80}, {0, -80}, {0, 32}}, color = {175, 175, 175}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Samplers;

      model Magnetometer
        inner RocketControl.World.MyWorld world(altitude_0 = 100, latitude_0 = 45, longitude_0 = 8) annotation(
          Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
          Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Real b[3];
        RocketControl.Components.Sensors.IdealMagnetometer idealMagnetometer annotation(
          Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        b = world.magneticField(fixed.frame_b.r_0) * 1e9;
        connect(fixed.frame_b, idealMagnetometer.frame_a) annotation(
          Line(points = {{-40, 0}, {-12, 0}}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Magnetometer;

      model RealGyroBias
        RocketControl.Components.Sensors.RealGyroscope realGyroscope(bias = {3, -2, 1}, bits = 16, fixedLocalSeed = {9, 423, 43214321}, rate_max = 250, samplePeriodMs = 20, sigmaARW = 0.5, sigmaRRW = 1) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
          Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(fixed.frame_b, realGyroscope.frame_a) annotation(
          Line(points = {{-40, 0}, {-10, 0}}, color = {95, 95, 95}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end RealGyroBias;

      model DifferentSampleFreqNotWorking
      RocketControl.Tests.MotionFrame motionFrame(v0 = {100, 0, 0}, w(each displayUnit = "deg/s") = {0.03490658503988657, 0.05235987755982988, 0.06981317007977318})  annotation(
          Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler eul_real annotation(
          Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion quaternion annotation(
          Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealMagnetometer idealMagnetometer annotation(
          Placement(visible = true, transformation(origin = {-48, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealGyroscope idealGyroscope annotation(
          Placement(visible = true, transformation(origin = {-48, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.AttitudeEstimation attitudeEstimation(elevation0 = 0, heading0 = 0, samplingPeriodMs = 100, sigma_b = 1, sigma_u = 1, sigma_v = 1)  annotation(
          Placement(visible = true, transformation(origin = {92, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
          Placement(visible = true, transformation(origin = {-48, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock clock1(factor = 100)  annotation(
          Placement(visible = true, transformation(origin = {-30, -86}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.RealSignals.Sampler.SampleVectorizedAndClocked sample1(n = 3)  annotation(
          Placement(visible = true, transformation(origin = {2, 20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.RealSignals.Sampler.SampleVectorizedAndClocked sampleVectorizedAndClocked(n = 3) annotation(
          Placement(visible = true, transformation(origin = {2, -52}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.RealSignals.Sampler.SampleVectorizedAndClocked sample11(n = 3)  annotation(
          Placement(visible = true, transformation(origin = {2, -20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicExactClock(factor = 60) annotation(
          Placement(visible = true, transformation(origin = {-20, -36}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler eul_est annotation(
          Placement(visible = true, transformation(origin = {132, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.Resample resample(nu = 3, samplePeriodMS = 100)  annotation(
          Placement(visible = true, transformation(origin = {50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.Resample resample1(nu = 3, samplePeriodMS = 100) annotation(
          Placement(visible = true, transformation(origin = {50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.Resample resample2(nu = 3, samplePeriodMS = 100) annotation(
          Placement(visible = true, transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(motionFrame.frame_b, quaternion.frame_a) annotation(
          Line(points = {{-80, 0}, {-72, 0}, {-72, 90}, {-60, 90}}));
        connect(quaternion.q, eul_real.q) annotation(
          Line(points = {{-40, 90}, {-22, 90}}, color = {0, 0, 127}, thickness = 0.5));
  connect(idealMagnetometer.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-58, -20}, {-72, -20}, {-72, 0}, {-80, 0}}, color = {95, 95, 95}));
  connect(idealGyroscope.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-58, 20}, {-59, 20}, {-59, 0}, {-80, 0}}, color = {95, 95, 95}));
  connect(absolutePosition.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-58, -52}, {-72, -52}, {-72, 0}, {-80, 0}}, color = {95, 95, 95}));
  connect(idealGyroscope.w, sample1.u) annotation(
          Line(points = {{-37.8, 20}, {-5.8, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absolutePosition.r, sampleVectorizedAndClocked.u) annotation(
          Line(points = {{-37, -52}, {-7, -52}}, color = {0, 0, 127}, thickness = 0.5));
  connect(idealMagnetometer.b, sample11.u) annotation(
          Line(points = {{-37.6, -20}, {-5.6, -20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(clock1.y, sampleVectorizedAndClocked.clock) annotation(
          Line(points = {{-23.4, -86}, {22.6, -86}, {22.6, -60}, {2.6, -60}}, color = {175, 175, 175}));
  connect(clock1.y, sample1.clock) annotation(
          Line(points = {{-23.4, -86}, {22.6, -86}, {22.6, 12}, {2.6, 12}}, color = {175, 175, 175}));
  connect(periodicExactClock.y, sample11.clock) annotation(
          Line(points = {{-13.4, -36}, {1.6, -36}, {1.6, -28}}, color = {175, 175, 175}));
  connect(attitudeEstimation.q_est, eul_est.q) annotation(
          Line(points = {{103, -5}, {129, -5}, {129, -10}, {120, -10}}, color = {0, 0, 127}, thickness = 0.5));
  connect(sample1.y, resample.u) annotation(
          Line(points = {{8, 20}, {38, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(resample.y, attitudeEstimation.w_meas_degs) annotation(
          Line(points = {{62, 20}, {80, 20}, {80, -2}}, color = {0, 0, 127}, thickness = 0.5));
  connect(sampleVectorizedAndClocked.y, resample2.u) annotation(
          Line(points = {{8, -52}, {38, -52}, {38, -50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(resample2.y, attitudeEstimation.r_0_est) annotation(
          Line(points = {{62, -50}, {80, -50}, {80, -18}}, color = {0, 0, 127}, thickness = 0.5));
  connect(resample1.y, attitudeEstimation.b_meas_nt) annotation(
          Line(points = {{62, -10}, {80, -10}}, color = {0, 0, 127}, thickness = 0.5));
  connect(sample11.y, resample1.u) annotation(
          Line(points = {{8, -20}, {38, -20}, {38, -10}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end DifferentSampleFreqNotWorking;
      
      model DifferentSampleFreqWorking
      RocketControl.Tests.MotionFrame motionFrame(v0 = {100, 0, 0}, w(each displayUnit = "deg/s") = {0.03490658503988657, 0.05235987755982988, 0.06981317007977318})  annotation(
          Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Math.Blocks.Quaternion2Euler eul_real annotation(
          Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Math.Blocks.Quaternion quaternion annotation(
          Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Sensors.IdealMagnetometer idealMagnetometer annotation(
          Placement(visible = true, transformation(origin = {-48, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Components.Sensors.IdealGyroscope idealGyroscope annotation(
          Placement(visible = true, transformation(origin = {-48, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.GNC.Navigation.AttitudeEstimation attitudeEstimation(elevation0 = 0, heading0 = 0, samplingPeriodMs = 60, sigma_b = 1, sigma_u = 1, sigma_v = 1)  annotation(
          Placement(visible = true, transformation(origin = {50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
          Placement(visible = true, transformation(origin = {-48, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock clock1(factor = 100)  annotation(
          Placement(visible = true, transformation(origin = {-30, -86}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Clocked.RealSignals.Sampler.SampleVectorizedAndClocked sample1(n = 3)  annotation(
          Placement(visible = true, transformation(origin = {2, 20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Clocked.RealSignals.Sampler.SampleVectorizedAndClocked sampleVectorizedAndClocked(n = 3) annotation(
          Placement(visible = true, transformation(origin = {2, -52}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Clocked.RealSignals.Sampler.SampleVectorizedAndClocked sample11(n = 3)  annotation(
          Placement(visible = true, transformation(origin = {2, -20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicExactClock(factor = 60) annotation(
          Placement(visible = true, transformation(origin = {-20, -36}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      RocketControl.Math.Blocks.Quaternion2Euler eul_est annotation(
          Placement(visible = true, transformation(origin = {90, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(motionFrame.frame_b, quaternion.frame_a) annotation(
          Line(points = {{-80, 0}, {-72, 0}, {-72, 90}, {-60, 90}}));
        connect(quaternion.q, eul_real.q) annotation(
          Line(points = {{-40, 90}, {-22, 90}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealMagnetometer.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-58, -20}, {-72, -20}, {-72, 0}, {-80, 0}}, color = {95, 95, 95}));
      connect(idealGyroscope.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-58, 20}, {-59, 20}, {-59, 0}, {-80, 0}}, color = {95, 95, 95}));
      connect(absolutePosition.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-58, -52}, {-72, -52}, {-72, 0}, {-80, 0}}, color = {95, 95, 95}));
      connect(idealGyroscope.w, sample1.u) annotation(
          Line(points = {{-37.8, 20}, {-5.8, 20}}, color = {0, 0, 127}, thickness = 0.5));
      connect(sample1.y, attitudeEstimation.w_meas_degs) annotation(
          Line(points = {{8.6, 20}, {38, 20}, {38, -2}}, color = {0, 0, 127}, thickness = 0.5));
      connect(absolutePosition.r, sampleVectorizedAndClocked.u) annotation(
          Line(points = {{-37, -52}, {-7, -52}}, color = {0, 0, 127}, thickness = 0.5));
      connect(sampleVectorizedAndClocked.y, attitudeEstimation.r_0_est) annotation(
          Line(points = {{8.6, -52}, {38.6, -52}, {38.6, -18}, {38, -18}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealMagnetometer.b, sample11.u) annotation(
          Line(points = {{-37.6, -20}, {-5.6, -20}}, color = {0, 0, 127}, thickness = 0.5));
      connect(sample11.y, attitudeEstimation.b_meas_nt) annotation(
          Line(points = {{8.6, -20}, {28.6, -20}, {28.6, -10}, {38, -10}}, color = {0, 0, 127}, thickness = 0.5));
      connect(clock1.y, sampleVectorizedAndClocked.clock) annotation(
          Line(points = {{-23.4, -86}, {22.6, -86}, {22.6, -60}, {2.6, -60}}, color = {175, 175, 175}));
      connect(clock1.y, sample1.clock) annotation(
          Line(points = {{-23.4, -86}, {22.6, -86}, {22.6, 12}, {2.6, 12}}, color = {175, 175, 175}));
      connect(periodicExactClock.y, sample11.clock) annotation(
          Line(points = {{-13.4, -36}, {1.6, -36}, {1.6, -28}}, color = {175, 175, 175}));
  connect(attitudeEstimation.q_est, eul_est.q) annotation(
          Line(points = {{62, -4}, {68, -4}, {68, -14}, {78, -14}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end DifferentSampleFreqWorking;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Sensors;

    package World

      model MagneticField
        RocketControl.World.MyWorld world(altitude_0 = 100, latitude_0 = 45, longitude_0 = 8) annotation(
          Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Real b[3];
      equation
        b = world.magneticField({500000, 500000, 500000});
        annotation(
          Icon(coordinateSystem(grid = {2, 0})),
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
      end MagneticField;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end World;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Components;
end Tests;
