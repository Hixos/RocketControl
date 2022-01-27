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
        inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(
          Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Tests.MotionFrame motionFrame(v0 = {0, 0, 0}, w(each displayUnit = "deg/s") = {0.08726646259971647, 0.1745329251994329, -0.08726646259971647}) annotation(
          Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealGyroscope idealGyroscope(samplePeriodMs = 20)  annotation(
          Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealGyroscope realGyroscope(bias = {0.00174532925199433, 0.003490658503988659, -0.006981317007977318}, bits = 16, rate_max = 4.363323129985824, samplePeriodMs = 20, sigmaARW = 0.00174532925199433, sigmaRRW = 0.000174532925199433)  annotation(
          Placement(visible = true, transformation(origin = {-10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(idealGyroscope.frame_a, motionFrame.frame_b) annotation(
          Line(points = {{-20, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(motionFrame.frame_b, realGyroscope.frame_a) annotation(
          Line(points = {{-60, 0}, {-38, 0}, {-38, 40}, {-20, 40}}));
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

    package Blocks
      model Track
      RocketControl.Components.Blocks.Track track annotation(
          Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
          Placement(visible = true, transformation(origin = {-50, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp ramp(duration = 10, height = 2 * pi)  annotation(
          Placement(visible = true, transformation(origin = {-90, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sin sin annotation(
          Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Cos cos annotation(
          Placement(visible = true, transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
          Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine(amplitude = 300, f = 2, offset = 300) annotation(
          Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product1 annotation(
          Placement(visible = true, transformation(origin = {-10, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.UnitConversions.To_deg track_true annotation(
          Placement(visible = true, transformation(origin = {72, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.WrapAngle wrapAngle annotation(
          Placement(visible = true, transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(ramp.y, sin.u) annotation(
          Line(points = {{-78, 12}, {-70, 12}, {-70, -10}, {-62, -10}}, color = {0, 0, 127}));
        connect(ramp.y, cos.u) annotation(
          Line(points = {{-78, 12}, {-70, 12}, {-70, 30}, {-62, 30}}, color = {0, 0, 127}));
        connect(sin.y, product1.u2) annotation(
          Line(points = {{-38, -10}, {-22, -10}, {-22, -16}}, color = {0, 0, 127}));
        connect(cos.y, product.u2) annotation(
          Line(points = {{-38, 30}, {-22, 30}, {-22, 24}}, color = {0, 0, 127}));
        connect(product1.u1, product.u1) annotation(
          Line(points = {{-22, -4}, {-22, 36}}, color = {0, 255, 255}));
        connect(product.y, track.v[1]) annotation(
          Line(points = {{2, 30}, {38, 30}, {38, 0}}, color = {0, 0, 127}));
        connect(product1.y, track.v[2]) annotation(
          Line(points = {{2, -10}, {38, -10}, {38, 0}}, color = {0, 0, 127}));
        connect(const.y, track.v[3]) annotation(
          Line(points = {{-38, -42}, {38, -42}, {38, 0}}, color = {0, 0, 127}));
        connect(ramp.y, wrapAngle.u) annotation(
          Line(points = {{-78, 12}, {-68, 12}, {-68, 50}, {18, 50}}, color = {0, 0, 127}));
        connect(wrapAngle.y, track_true.u) annotation(
          Line(points = {{42, 50}, {60, 50}}, color = {0, 0, 127}));
  connect(sine.y, product.u1) annotation(
          Line(points = {{-78, 90}, {-22, 90}, {-22, 36}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Track;

      model EulerRatesTest
      RocketControl.Components.Blocks.EulerRates eulerRates annotation(
          Placement(visible = true, transformation(origin = {80, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Continuous.IdealAsset idealAsset annotation(
          Placement(visible = true, transformation(origin = {4, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
          Placement(visible = true, transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Continuous.IdealGyroscope idealGyroscope annotation(
          Placement(visible = true, transformation(origin = {0, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {-32, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity)  annotation(
          Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(angle_1(fixed = true), angle_2(fixed = true, start = from_deg(90)), angle_3(fixed = true), sequence_start = {3, 2, 1},use_w = true, w_rel_b_1(fixed = true, start = from_deg(45)), w_rel_b_2(fixed = true, start = from_deg(45)), w_rel_b_3(fixed = true, start = 0))  annotation(
          Placement(visible = true, transformation(origin = {-60, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler quaternion2Euler annotation(
          Placement(visible = true, transformation(origin = {52, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {1, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {-12, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(idealAsset.q, eulerRates.q) annotation(
          Line(points = {{15, -18}, {41, -18}, {41, 12}, {67, 12}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGyroscope.w, eulerRates.w) annotation(
          Line(points = {{10.2, 56}, {44.2, 56}, {44.2, 24}, {68.2, 24}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixed.frame_b, freeMotionScalarInit.frame_a) annotation(
          Line(points = {{-80, -30}, {-70, -30}}, color = {95, 95, 95}));
  connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
          Line(points = {{-50, -30}, {-42, -30}}, color = {95, 95, 95}));
  connect(idealAsset.frame_a, body.frame_a) annotation(
          Line(points = {{-6, -18}, {-42, -18}, {-42, -30}}));
  connect(idealGyroscope.frame_a, body.frame_a) annotation(
          Line(points = {{-10, 56}, {-42, 56}, {-42, -30}}));
  connect(idealAsset.q, quaternion2Euler.q) annotation(
          Line(points = {{16, -18}, {28, -18}, {28, -62}, {40, -62}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedTranslation.frame_a, body.frame_a) annotation(
          Line(points = {{-22, 12}, {-42, 12}, {-42, -30}}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end EulerRatesTest;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Blocks;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Components;

  package GNC
    package Control
      model Autopilot
      RocketControl.GNC.Control.Autopilot autopilot(CLLdr = 0.0388, CLMalpha = -0.6798, CLMdp = 0.3573, CLNbeta = 0.6798, CLNdy = 0.3573, CNalpha = 0.4158, CNdp = 0.0557, CYbeta = -0.4158, CYdy = 0.0557, S = 0.01, c = 0.002, fin_max_angle = 10) annotation(
          Placement(visible = true, transformation(origin = {90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
          Placement(visible = true, transformation(origin = {30, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp fdir(duration = 20, height = 360)  annotation(
          Placement(visible = true, transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sin sin annotation(
          Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Cos cos annotation(
          Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
          Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product1 annotation(
          Placement(visible = true, transformation(origin = {10, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.UnitConversions.From_deg from_deg annotation(
          Placement(visible = true, transformation(origin = {-60, 30}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Math.UnitConversions.From_deg from_deg1 annotation(
          Placement(visible = true, transformation(origin = {-58, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 0.5 * 200 ^ 2)  annotation(
          Placement(visible = true, transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = 300) annotation(
          Placement(visible = true, transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(fdir.y, from_deg.u) annotation(
          Line(points = {{-78, 30}, {-64, 30}}, color = {0, 0, 127}));
        connect(from_deg.y, cos.u) annotation(
          Line(points = {{-56, 30}, {-42, 30}}, color = {0, 0, 127}));
        connect(from_deg1.y, sin.u) annotation(
          Line(points = {{-54, -10}, {-42, -10}}, color = {0, 0, 127}));
        connect(from_deg1.u, fdir.y) annotation(
          Line(points = {{-62, -10}, {-72, -10}, {-72, 30}, {-78, 30}}, color = {0, 0, 127}));
        connect(cos.y, product.u2) annotation(
          Line(points = {{-18, 30}, {-12, 30}, {-12, 24}, {-2, 24}}, color = {0, 0, 127}));
        connect(sin.y, product1.u2) annotation(
          Line(points = {{-18, -10}, {-12, -10}, {-12, -16}, {-2, -16}}, color = {0, 0, 127}));
        connect(product.y, autopilot.Fn) annotation(
          Line(points = {{22, 30}, {48, 30}, {48, 18}, {78, 18}}, color = {0, 0, 127}));
        connect(product1.y, autopilot.Fy) annotation(
          Line(points = {{22, -10}, {48, -10}, {48, 14}, {78, 14}}, color = {0, 0, 127}));
        connect(const.y, autopilot.Ml) annotation(
          Line(points = {{42, -50}, {60, -50}, {60, 10}, {78, 10}}, color = {0, 0, 127}));
        connect(constant1.y, autopilot.q) annotation(
          Line(points = {{42, -90}, {68, -90}, {68, 2}, {78, 2}}, color = {0, 0, 127}));
  connect(constant2.y, product.u1) annotation(
          Line(points = {{-79, 70}, {-8, 70}, {-8, 36}, {-2, 36}}, color = {0, 0, 127}));
  connect(constant2.y, product1.u1) annotation(
          Line(points = {{-79, 70}, {-8, 70}, {-8, -4}, {-2, -4}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})),
          experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-6, Interval = 0.01));
      end Autopilot;

      model ContinuousLQTest
      RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -10,m = 1, n = 2)  annotation(
          Placement(visible = true, transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Matrix.MatrixConstant A(n = 2, val = [1, 1; 0, -1])  annotation(
          Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Matrix.MatrixConstant B(m = 1, n = 2, val = [1; 0])  annotation(
          Placement(visible = true, transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 2, val = [1, 0; 0, 1])  annotation(
          Placement(visible = true, transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Matrix.MatrixConstant R(val = [0.001])  annotation(
          Placement(visible = true, transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant x(k = {2, 1}, n = 2)  annotation(
          Placement(visible = true, transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(x.v, continuousLQR.x) annotation(
          Line(points = {{-58, -90}, {18, -90}, {18, 2}}, color = {0, 0, 127}, thickness = 0.5));
  connect(R.k, continuousLQR.R) annotation(
          Line(points = {{-58, -50}, {0, -50}, {0, 6}, {18, 6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{-58, -10}, {-24, -10}, {-24, 10}, {18, 10}}, color = {0, 0, 127}, thickness = 0.5));
  connect(B.k, continuousLQR.B) annotation(
          Line(points = {{-58, 30}, {-24, 30}, {-24, 14}, {18, 14}}, color = {0, 0, 127}, thickness = 0.5));
  connect(A.k, continuousLQR.A) annotation(
          Line(points = {{-58, 70}, {-10, 70}, {-10, 18}, {18, 18}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end ContinuousLQTest;

      model SystemMatrixtest
      RocketControl.GNC.Control.LinearLQ.SystemMatrices systemMatrices(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 1, Ix = 0.1, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 25)  annotation(
          Placement(visible = true, transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {150, 5, -10})  annotation(
          Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant1(k = {1, -1, -0.5}) annotation(
          Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 1)  annotation(
          Placement(visible = true, transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(vectorConstant.v, systemMatrices.vel) annotation(
          Line(points = {{-59, 50}, {-30, 50}, {-30, 6}, {-10, 6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConstant1.v, systemMatrices.ang_vel) annotation(
          Line(points = {{-59, 0}, {-10, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const.y, systemMatrices.rho) annotation(
          Line(points = {{-59, -50}, {-30, -50}, {-30, -6}, {-10, -6}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end SystemMatrixtest;

      model AngularRateReferenceTest
      
      Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
          Placement(visible = true, transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation(angles = {45, 0, 0}, rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence, sequence = {3, 2, 1})  annotation(
          Placement(visible = true, transformation(origin = {-64, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant vt(k = {1, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {-90, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant v_body(k = {200, 0, -10}) annotation(
          Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Continuous.IdealAsset idealAsset annotation(
          Placement(visible = true, transformation(origin = {-28, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.AngularRateToTarget angularRateToTarget(k = 1)  annotation(
          Placement(visible = true, transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(fixed.frame_b, fixedRotation.frame_a) annotation(
          Line(points = {{-80, -30}, {-74, -30}}, color = {95, 95, 95}));
        connect(fixedRotation.frame_b, idealAsset.frame_a) annotation(
          Line(points = {{-54, -30}, {-38, -30}}, color = {95, 95, 95}));
  connect(vt.v, angularRateToTarget.vt_ned) annotation(
          Line(points = {{-78, 32}, {-50, 32}, {-50, 50}, {18, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(idealAsset.q, angularRateToTarget.q) annotation(
          Line(points = {{-16, -30}, {4, -30}, {4, 26}, {-32, 26}, {-32, 44}, {18, 44}}, color = {0, 0, 127}, thickness = 0.5));
  connect(v_body.v, angularRateToTarget.v) annotation(
          Line(points = {{-78, 90}, {-50, 90}, {-50, 56}, {18, 56}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end AngularRateReferenceTest;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Control;

    package Navigation
      model PositionEstimation
      RocketControl.GNC.Navigation.PositionEstimation positionEstimation2(samplingPeriodMs = 20, sigma_gps = {200000, 200000, 200000, 600000, 600000, 600000}, sigma_pos = 1, sigma_vel = 2, x0 = {0, 0, 0, 0, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {6, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, 1})  annotation(
          Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(angle_1(fixed = true), angle_2(fixed = true), angle_3(fixed = true), angle_d_1(fixed = true, start = 2), angle_d_2(fixed = true, start = 3), angle_d_3(fixed = true, start = -1), r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_angle = true, use_angle_d = true, use_r = true, use_v = true, v_rel_a_1(fixed = true), v_rel_a_2(fixed = true), v_rel_a_3(fixed = true, start = 0))  annotation(
          Placement(visible = true, transformation(origin = {-48, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealAccelerometer realAccelerometer(acc_max = 150, bias = {0, 0, 0}, bits = 16, samplePeriodMs = 20, sigmaBiasInstability = 0, sigmaNoise = 1)  annotation(
          Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.IdealAttitudeSampled idealAttitudeSampled(samplingPeriodMs = 20)  annotation(
          Placement(visible = true, transformation(origin = {-30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealGNSS realGNSS(samplePeriodMs = 20, sigmaNoise_vxy = 3, sigmaNoise_vz = 5, sigmaNoise_xy = 30, sigmaNoise_z = 50)  annotation(
          Placement(visible = true, transformation(origin = {-32, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(freeMotionScalarInit.frame_a, world.frame_b) annotation(
          Line(points = {{-58, -58}, {-80, -58}, {-80, -90}}, color = {95, 95, 95}));
  connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
          Line(points = {{-38, -58}, {-4, -58}}, color = {95, 95, 95}));
  connect(realAccelerometer.acc, positionEstimation2.acc_body) annotation(
          Line(points = {{-20, 30}, {8, 30}, {8, 53}, {38, 53}}, color = {0, 0, 127}, thickness = 0.5));
  connect(idealAttitudeSampled.q, positionEstimation2.q) annotation(
          Line(points = {{-19, 70}, {8.5, 70}, {8.5, 58}, {38, 58}}, color = {0, 0, 127}, thickness = 0.5));
  connect(idealAttitudeSampled.frame_a, body.frame_a) annotation(
          Line(points = {{-40, 70}, {-52, 70}, {-52, -20}, {-20, -20}, {-20, -58}, {-4, -58}}, color = {95, 95, 95}));
  connect(realAccelerometer.frame_a, body.frame_a) annotation(
          Line(points = {{-40, 30}, {-52, 30}, {-52, -20}, {-20, -20}, {-20, -58}, {-4, -58}}, color = {95, 95, 95}));
  connect(realGNSS.frame_a, body.frame_a) annotation(
          Line(points = {{-42, 0}, {-52, 0}, {-52, -20}, {-20, -20}, {-20, -58}, {-4, -58}}, color = {95, 95, 95}));
  connect(realGNSS.pos, positionEstimation2.pos_ned) annotation(
          Line(points = {{-20, 4}, {14, 4}, {14, 47}, {38, 47}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realGNSS.vel, positionEstimation2.vel_ned) annotation(
          Line(points = {{-20, -4}, {22, -4}, {22, 42}, {38, 42}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end PositionEstimation;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Navigation;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end GNC;

  package Aerodynamics
    model AeroAnglesSensor
    RocketControl.Aerodynamics.AeroAnglesSensor aeroAnglesSensor annotation(
        Placement(visible = true, transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldTorque torque annotation(
        Placement(visible = true, transformation(origin = {-66, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion quaternion annotation(
        Placement(visible = true, transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler quaternion2Euler annotation(
        Placement(visible = true, transformation(origin = {48, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 1, I_22 = 1, I_33 = 1, m = 10, r_CM = {0, 0, 0})  annotation(
        Placement(visible = true, transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const(k = 0.1)  annotation(
        Placement(visible = true, transformation(origin = {-118, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 0)  annotation(
        Placement(visible = true, transformation(origin = {-118, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.MyWorld world(n = {0, 0, 1})  annotation(
        Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(
        Placement(visible = true, transformation(origin = {2, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(use_v = true, v_rel_a_1(fixed = true, start = 10), v_rel_a_2(fixed = true), v_rel_a_3(fixed = true))  annotation(
        Placement(visible = true, transformation(origin = {-108, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
  connect(quaternion.frame_a, aeroAnglesSensor.frame_a) annotation(
        Line(points = {{0, 50}, {-8, 50}, {-8, 0}}, color = {95, 95, 95}));
  connect(quaternion2Euler.q, quaternion.q) annotation(
        Line(points = {{36, 50}, {20, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(body.frame_a, aeroAnglesSensor.frame_a) annotation(
        Line(points = {{-50, 20}, {-37, 20}, {-37, 0}, {-8, 0}}, color = {95, 95, 95}));
  connect(torque.frame_b, body.frame_a) annotation(
        Line(points = {{-56, -38}, {-56, -8}, {-50, -8}, {-50, 20}}));
  connect(const1.y, torque.torque[1]) annotation(
        Line(points = {{-106, -54}, {-78, -54}, {-78, -38}}, color = {0, 0, 127}));
  connect(const1.y, torque.torque[3]) annotation(
        Line(points = {{-106, -54}, {-78, -54}, {-78, -38}}, color = {0, 0, 127}));
  connect(const.y, torque.torque[2]) annotation(
        Line(points = {{-106, -20}, {-78, -20}, {-78, -38}}, color = {0, 0, 127}));
  connect(absoluteVelocity.frame_a, body.frame_a) annotation(
        Line(points = {{-8, -82}, {-50, -82}, {-50, 20}}));
  connect(freeMotionScalarInit.frame_a, world.frame_b) annotation(
        Line(points = {{-118, 50}, {-80, 50}, {-80, -90}}, color = {95, 95, 95}));
  connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
        Line(points = {{-98, 50}, {-50, 50}, {-50, 20}}, color = {95, 95, 95}));
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AeroAnglesSensor;

    model AeroOpt
      Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
        Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
        Placement(visible = true, transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
        Placement(visible = true, transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Tests.Aerodynamics.Propagate propagate(n = 4)  annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
        Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = 1) annotation(
        Placement(visible = true, transformation(origin = {-68, -78}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.RealSignals.Sampler.SampleClocked sample1 annotation(
        Placement(visible = true, transformation(origin = {-58, -30}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.RealSignals.Sampler.Hold hold1 annotation(
        Placement(visible = true, transformation(origin = {-18, -30}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      connect(fixed.frame_b, aerodynamics.frame_b) annotation(
        Line(points = {{-80, 30}, {80, 30}}));
      connect(propagate.y, aerodynamics.finDeflection) annotation(
        Line(points = {{41, -30}, {80, -30}, {80, 24}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const.y, sample1.u) annotation(
        Line(points = {{-78, -30}, {-66, -30}}, color = {0, 0, 127}));
  connect(periodicClock1.y, sample1.clock) annotation(
        Line(points = {{-62, -78}, {-58, -78}, {-58, -38}}, color = {175, 175, 175}));
  connect(sample1.y, hold1.u) annotation(
        Line(points = {{-52, -30}, {-25, -30}}, color = {0, 0, 127}));
  connect(hold1.y, propagate.u) annotation(
        Line(points = {{-11, -30}, {18, -30}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AeroOpt;

    model Propagate
    parameter Integer n(min = 1) = 1 annotation(Evaluate = true);
    
    Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y[n] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
  for i in 1:n loop
    y[i] = u;
      end for;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Propagate;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Aerodynamics;

  model World
    import RocketControl.Types.NanoTesla;
    parameter NonSI.Angle_deg latitude_0 = 41.808682 "Latitude of the origin of the world frame";
    parameter NonSI.Angle_deg longitude_0 = 14.054440 "Longitude of the origin of the world frame";
    parameter SI.Position altitude_0 = 1416 "Altitude of the origin of the world frame";
    parameter SI.Length a = 6371000 "Planetary mean radius";
    parameter NonSI.Angle_deg pole_lat = 80.59 "North geomagnetic pole latitude";
    parameter NonSI.Angle_deg pole_lon = -72.64 "North geomagnetic pole longitude";
    parameter NanoTesla g10 = -29404.8 "IGRF gauss coefficient (n=1, m=0)";
    parameter NanoTesla g11 = -1450.9 "IGRF gauss coefficient (n=1, m=1)";
    parameter NanoTesla h11 = 4652.5 "IGRF gauss coefficient (n=1, m=1)";
    parameter SI.Position x_0_ecef[3] = RocketControl.World.MyWorld.lla2ecef(latitude_0, longitude_0, altitude_0) "Coordinates of the origin of the NED (world) frame in the ecef frame";
    parameter Real T[3, 3] = RocketControl.World.MyWorld.ecef2nedMatrix(x_0_ecef) "Transformation matrix from world frame to ned frame";
    //    final parameter Real rpole[3] = Coordinates.lla2ecef(pole_lat, pole_lon, 0) "Magnetic north dipole pole coordinates in ecef frame (IGRF 2020)";
    //    final parameter Real m[3] = -rpole / norm(rpole) "Magnetic dipole axis in ecef frame (IGRF 2020)";
    final parameter SI.Length R = 6371200 "IGRF Earth radius";
    final parameter SI.MagneticFluxDensity H0 = sqrt(g10 ^ 2 + g11 ^ 2 + h11 ^ 2) / 1e9;
    final parameter Real m_vec[3] = -RocketControl.World.MyWorld.lla2ecef(pole_lat, pole_lon, 0);
    final parameter Real m[3] = m_vec / norm(m_vec);
  equation

    annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end World;

  package Math
    model sdaCare
    parameter Real A[5,5] = [0, 0, 1.0000e+00, 0, 0; 0, 0, 0, 1.0000e+00, 0; -6.5400e+01, 6.5400e+01, -3.2700e+00, 3.2700e+00, 0; 1.9620e+03, -6.2784e+04, 9.8100e+01, -9.8100e+01, 0; -1.0000e+00, 0, 0, 0, 0];
    parameter Real B[5,1] = [0; 0; 1.66666666666667e-03; -5e-02; 0];
    
    parameter Real Q[5,5] = diagonal({0,0,0,0,1});
    parameter Real R[1,1] = [0.001];
    
    parameter Real P[5,5] = RocketControl.Math.sdaCare(A,B,R,Q);
    equation

      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end sdaCare;
    
    model sdaDareTest
    parameter Real dt = 0.001;
    parameter Real At[4,4] = [1, 0, 0.001, 0; 0, 1, 0, 0.001; -0.0654, 0.0654, 0.99673, 0.00327; 1.962, -62.784, 0.0981, 0.9019];
    parameter Real B[5,1] = [0; 0; 1.66666666666667e-06; -5e-05; 0];
    parameter Real C[1,4] = [1,0,0,0];
    
    parameter Real A[5,5] = [At, zeros(4,1); -C*dt, 1];
    
    parameter Real Q[5,5] = diagonal({0,0,0,0,1});
    parameter Real R[1,1] = [0.001];
    
    parameter Real P[5,5] = RocketControl.Math.sdaDare(A,B,R,Q);
    equation
    
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end sdaDareTest;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Math;

  model angularvelocity
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0})  annotation(
      Placement(visible = true, transformation(origin = {-10, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_r = true,use_v = true, use_w = true, v_rel_a_1(fixed = true, start = v[1]), v_rel_a_2(fixed = true, start = v[2]), v_rel_a_3(fixed = true, start = v[3]), w_rel_b_1(fixed = true, start = w[1]), w_rel_b_2(fixed = true, start = w[2]), w_rel_b_3(fixed = true, start = w[3]))  annotation(
      Placement(visible = true, transformation(origin = {-64, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
      Placement(visible = true, transformation(origin = {-112, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity)  annotation(
      Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a)  annotation(
      Placement(visible = true, transformation(origin = {-10, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      
      parameter Real w[3] = {0,0,1};
      parameter Real v[3] = {1,0,0};
      
      Real v_body_calc[3];
      
      Real acc_calc[3];
      Real acc_meas[3];
      
      Real Abw[3,3];
      Real t[3];
  equation
    t = min({1, 2, 3}, {1.5,1.5,1});
     Abw = body.frame_a.R.T;
     v_body_calc = Abw*v;
     
     acc_meas = der(absoluteVelocity.v);
     acc_calc = cross(w, absoluteVelocity.v);
    connect(fixed.frame_b, freeMotionScalarInit.frame_a) annotation(
      Line(points = {{-102, -6}, {-74, -6}}, color = {95, 95, 95}));
    connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
      Line(points = {{-54, -6}, {-20, -6}}, color = {95, 95, 95}));
  connect(absoluteVelocity.frame_a, body.frame_a) annotation(
      Line(points = {{-20, 60}, {-28, 60}, {-28, -6}, {-20, -6}}, color = {95, 95, 95}));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end angularvelocity;
end Tests;
