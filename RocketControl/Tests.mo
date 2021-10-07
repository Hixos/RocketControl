within RocketControl;

package Tests
  model SensorTests
    import Modelica.Units.Conversions.from_deg;
    import Modelica.Units.SI;
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(guessAngle1 = 0, sequence = {3, 2, 1}) annotation(
      Placement(visible = true, transformation(origin = {62, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Tests.MotionFrame motionFrame(angles0 = from_deg({0, 0, 0}), v0 = {200, 0, 10}, w(each displayUnit = "deg/s") = from_deg({0, 0, 0})) annotation(
      Placement(visible = true, transformation(origin = {-32, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Sensors.AeroStateSensor aeroStateSensor annotation(
      Placement(visible = true, transformation(origin = {64, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
//  Connections.root(frame_a.R);
//  der(angles) = {0, 30, 0};
//  frame_a.R = Modelica.Mechanics.MultiBody.Frames.axesRotations({3, 2, 1}, angles, der(angles));
//  der(frame_a.r_0) = v0;
    connect(motionFrame.frame_b, absoluteAngles.frame_a) annotation(
      Line(points = {{-22, 8}, {52, 8}}, color = {95, 95, 95}));
    connect(aeroStateSensor.frame_a, motionFrame.frame_b) annotation(
      Line(points = {{54, -22}, {16, -22}, {16, 8}, {-22, 8}}, color = {95, 95, 95}));
  end SensorTests;

  model AerodynamicsTest
    import Modelica.Units.Conversions.from_deg;
    Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
      Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    RocketControl.Interfaces.RealAeroState realAeroState annotation(
      Placement(visible = true, transformation(origin = {4, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant h(k = 1000) annotation(
      Placement(visible = true, transformation(origin = {-76, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant v[3](k = {100, 0, 0}) annotation(
      Placement(visible = true, transformation(origin = {-76, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant w[3](k = from_deg({0, 0, 0})) annotation(
      Placement(visible = true, transformation(origin = {-76, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant m(k = 0.5) annotation(
      Placement(visible = true, transformation(origin = {-76, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant beta(k = from_deg(0)) annotation(
      Placement(visible = true, transformation(origin = {-76, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant alpha(k = from_deg(0)) annotation(
      Placement(visible = true, transformation(origin = {-76, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Aerodynamics.WithoutControl.AerodynamicForce aerodynamicForce annotation(
      Placement(visible = true, transformation(origin = {54, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(realAeroState.out, aerodynamics.aeroState) annotation(
      Line(points = {{24, 0}, {40, 0}}));
    connect(v.y, realAeroState.v) annotation(
      Line(points = {{-64, -54}, {-26, -54}, {-26, -6}, {-5, -6}}, color = {0, 0, 127}));
    connect(w.y, realAeroState.w) annotation(
      Line(points = {{-64, -84}, {-22, -84}, {-22, -9}, {-5, -9}}, color = {0, 0, 127}));
    connect(h.y, realAeroState.altitude) annotation(
      Line(points = {{-64, -22}, {-28, -22}, {-28, -2}, {-5, -2}}, color = {0, 0, 127}));
    connect(m.y, realAeroState.mach) annotation(
      Line(points = {{-64, 12}, {-28, 12}, {-28, 2}, {-5, 2}}, color = {0, 0, 127}));
    connect(beta.y, realAeroState.beta) annotation(
      Line(points = {{-65, 42}, {-26, 42}, {-26, 6}, {-5, 6}}, color = {0, 0, 127}));
    connect(alpha.y, realAeroState.alpha) annotation(
      Line(points = {{-65, 74}, {-22, 74}, {-22, 9}, {-5, 9}}, color = {0, 0, 127}));
    connect(realAeroState.out, aerodynamicForce.aeroState) annotation(
      Line(points = {{14, 0}, {44, 0}}));
    connect(aerodynamicForce.frame_b, fixed.frame_b) annotation(
      Line(points = {{64, 0}, {80, 0}}, color = {95, 95, 95}));
  end AerodynamicsTest;

  model SimpleAero
    import RocketControl.Aerodynamics.*;
    AeroData aerodata = AeroData();
    Real coeffs[Coeff];
    Real state_arr[State];
    Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(visible = true, transformation(origin = {-104, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-94, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    state_arr[State.alphas] = 10;
    state_arr[State.betas] = 0;
    state_arr[State.machs] = u / 300;
    state_arr[State.alts] = 500;
    coeffs = coeffValue(aerodata, state_arr);
    y = -0.5 * 1.225 * u ^ 2 * 0.08 * coeffs[Coeff.CA];
    annotation(
      Icon(graphics = {Ellipse(origin = {-1, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Horizontal, extent = {{-99, 100}, {99, -100}}, endAngle = 360)}));
  end SimpleAero;

  model MotorTest
    import Modelica.Units.Conversions.from_deg;
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Motors.M2000R m2000r annotation(
      Placement(visible = true, transformation(origin = {2, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  equation
    connect(world.frame_b, m2000r.frame_b) annotation(
      Line(points = {{-80, 10}, {-8, 10}}));
  end MotorTest;

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

  model VariableInertiaTest
    inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(
      Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Interfaces.MassPropertiesOutput massOutput annotation(
      Placement(visible = true, transformation(origin = {80, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-24, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    SI.Inertia I11;
    RocketControl.Components.BodyVariableMass bodyVariableMass(angles_fixed = true, r_0(each fixed = true), r_CM = {0, 0, 0}, v_0(each fixed = true), w_0_fixed = true, w_a(each fixed = true)) annotation(
      Placement(visible = true, transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.BodyVariableMass bodyVariableMass1(angles_fixed = true, r_0(each fixed = true), r_CM = {0, 0, 0}, v_0(each fixed = true), w_0_fixed = true, w_a(each fixed = true)) annotation(
      Placement(visible = true, transformation(origin = {30, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Interfaces.MassPropertiesOutput massOutput1 annotation(
      Placement(visible = true, transformation(origin = {80, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
      Placement(visible = true, transformation(origin = {-10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.WorldTorque torque annotation(
      Placement(visible = true, transformation(origin = {-10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = 1) annotation(
      Placement(visible = true, transformation(origin = {-76, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  initial equation
    I11 = 20;
    massOutput.m = 20;
  equation
    if massOutput.m > 1 then
      der(massOutput.m) = -1;
    else
      der(massOutput.m) = 0;
    end if;
    massOutput.I = [10, 0, 0; 0, 10, 0; 0, 0, 10];
    if I11 > 1 then
      der(I11) = -1;
    else
      der(I11) = 0;
    end if;
    massOutput1.m = 20;
    massOutput1.I = [I11, 0, 0; 0, 10, 0; 0, 0, 10];
    connect(massOutput, bodyVariableMass.massInput) annotation(
      Line(points = {{80, 20}, {34, 20}, {34, 34}}));
    connect(massOutput1, bodyVariableMass1.massInput) annotation(
      Line(points = {{80, -40}, {34, -40}, {34, -26}}));
    connect(force.frame_b, bodyVariableMass.frame_a) annotation(
      Line(points = {{0, 40}, {20, 40}}));
    connect(torque.frame_b, bodyVariableMass1.frame_a) annotation(
      Line(points = {{0, -20}, {20, -20}}, color = {95, 95, 95}));
    connect(const.y, force.force[1]) annotation(
      Line(points = {{-65, 36}, {-34, 36}, {-34, 40}, {-22, 40}}, color = {0, 0, 127}));
    connect(const.y, torque.torque[1]) annotation(
      Line(points = {{-65, 36}, {-34, 36}, {-34, -20}, {-22, -20}}, color = {0, 0, 127}));
    connect(constant1.y, torque.torque[2]) annotation(
      Line(points = {{-38, -30}, {-32, -30}, {-32, -20}, {-22, -20}}, color = {0, 0, 127}));
    connect(constant1.y, torque.torque[3]) annotation(
      Line(points = {{-38, -30}, {-30, -30}, {-30, -20}, {-22, -20}}, color = {0, 0, 127}));
    connect(constant1.y, force.force[2]) annotation(
      Line(points = {{-38, -30}, {-32, -30}, {-32, 40}, {-22, 40}}, color = {0, 0, 127}));
    connect(constant1.y, force.force[3]) annotation(
      Line(points = {{-38, -30}, {-28, -30}, {-28, 40}, {-22, 40}}, color = {0, 0, 127}));
  end VariableInertiaTest;

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
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Sensors;

    package World
      model wgs84
        parameter NonSI.Angle_deg lat = 0;
        parameter NonSI.Angle_deg lon = 0;
        parameter SI.Distance alt = 0;
        SI.Position pos_ecef[3];
        SI.Position pos_ned[3];
        NonSI.Angle_deg res_lat;
        NonSI.Angle_deg res_lon;
        SI.Position res_alt;
        Modelica.Mechanics.MultiBody.Frames.Orientation Rned;
        Real nadir[3];
      algorithm
        pos_ecef := RocketControl.World.MyWorld.WGS84.lla2ecef(lat, lon, alt);
        (res_lat, res_lon, res_alt) := RocketControl.World.MyWorld.WGS84.ecef2lla(pos_ecef);
        nadir := RocketControl.World.MyWorld.WGS84.getNadir(pos_ecef);
        Rned := RocketControl.World.MyWorld.NEDOrientation(pos_ecef);
        pos_ned := Modelica.Mechanics.MultiBody.Frames.resolve2(Rned, pos_ecef);
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end wgs84;

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

    package Frames
      model Ecef
        inner RocketControl.World.MyWorld world annotation(
          Placement(visible = true, transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Frames.Orientation R;
        RocketControl.Components.Frames.ECEF ecef annotation(
          Placement(visible = true, transformation(origin = {-72, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {6378137, 0, 0}) annotation(
          Placement(visible = true, transformation(origin = {-22, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(
          Placement(visible = true, transformation(origin = {74, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
          Placement(visible = true, transformation(origin = {74, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Real v2[3];
      equation
        v2 = RocketControl.World.MyWorld.eci2ecef(fixedTranslation.frame_b.r_0, time);
        R = RocketControl.World.MyWorld.eci2ecefOrientation(time);
        connect(ecef.frame_b, fixedTranslation.frame_a) annotation(
          Line(points = {{-62, 2}, {-32, 2}}, color = {95, 95, 95}));
        connect(fixedTranslation.frame_b, absoluteAngularVelocity.frame_a) annotation(
          Line(points = {{-12, 2}, {16, 2}, {16, 38}, {64, 38}}));
        connect(fixedTranslation.frame_b, absoluteVelocity.frame_a) annotation(
          Line(points = {{-12, 2}, {64, 2}}, color = {95, 95, 95}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Ecef;

      model Frames
        RocketControl.Components.Frames.ECEF ecef annotation(
          Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Frames.EarthSurfaceTranslation earthSurfaceTranslation(altitude = 309, latitude = 45.691049, longitude = 8.490597) annotation(
          Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        inner RocketControl.World.MyWorld world annotation(
          Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Frames.ecef2ned ecef2ned annotation(
          Placement(visible = true, transformation(origin = {48, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(ecef.frame_b, earthSurfaceTranslation.frame_a) annotation(
          Line(points = {{-60, 0}, {-20, 0}}, color = {95, 95, 95}));
        connect(earthSurfaceTranslation.frame_b, ecef2ned.frame_a) annotation(
          Line(points = {{0, 0}, {38, 0}}));
      protected
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Frames;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Frames;

    package Forces
      model ConnectionFlange
        Modelica.Mechanics.MultiBody.Parts.Body body(m = 20, r_CM = {0, 0, 0}) annotation(
          Placement(visible = true, transformation(origin = {72, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Forces.ConnectionFlange connectionFlange(c = 30000, d = 500, diameter = 0.09) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {1, 0, 0}) annotation(
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.UniformGravity) annotation(
          Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Parts.Body body1(m = 20, r_0(start = {100000, 100000, 100000}), r_CM = {0, 0, 0}) annotation(
          Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {1, 0, 0}) annotation(
          Placement(visible = true, transformation(origin = {-42, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Sensors.RelativePosition relativePosition annotation(
          Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Sensors.RelativeAngles relativeAngles annotation(
          Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(angle_1(fixed = true, start = 0), angle_2(fixed = true, start = 0), angle_3(fixed = true, start = 0), angle_d_1(fixed = true), angle_d_2(fixed = true), angle_d_3(fixed = true), r_rel_a_1(fixed = true, start = 2.001), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_angle = true, use_angle_d = true, use_r = true, use_v = true, v_rel_a_1(fixed = true), v_rel_a_2(fixed = true), v_rel_a_3(fixed = true)) annotation(
          Placement(visible = true, transformation(origin = {2, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(connectionFlange.frame_b, fixedTranslation.frame_a) annotation(
          Line(points = {{10, 0}, {26, 0}}, color = {95, 95, 95}));
        connect(fixedTranslation.frame_b, body.frame_a) annotation(
          Line(points = {{46, 0}, {62, 0}}));
        connect(fixedTranslation1.frame_b, connectionFlange.frame_a) annotation(
          Line(points = {{-32, 0}, {-10, 0}}, color = {95, 95, 95}));
        connect(body1.frame_a, fixedTranslation1.frame_a) annotation(
          Line(points = {{-70, 0}, {-52, 0}}));
        connect(relativePosition.frame_a, body1.frame_a) annotation(
          Line(points = {{-10, 80}, {-70, 80}, {-70, 0}}));
        connect(relativePosition.frame_b, body.frame_a) annotation(
          Line(points = {{10, 80}, {62, 80}, {62, 0}}, color = {95, 95, 95}));
        connect(relativeAngles.frame_a, body1.frame_a) annotation(
          Line(points = {{-10, 40}, {-70, 40}, {-70, 0}}, color = {95, 95, 95}));
        connect(relativeAngles.frame_b, body.frame_a) annotation(
          Line(points = {{10, 40}, {62, 40}, {62, 0}}, color = {95, 95, 95}));
        connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
          Line(points = {{12, -40}, {54, -40}, {54, 0}, {62, 0}}));
        connect(body1.frame_a, freeMotionScalarInit.frame_a) annotation(
          Line(points = {{-70, 0}, {-60, 0}, {-60, -40}, {-8, -40}}, color = {95, 95, 95}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end ConnectionFlange;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Forces;

    package Launchpad
      model RelativeInit
      Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
          Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {78, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(
          Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.LaunchPad.RelativePositionInit relativePositionInit(rel_angle_1(displayUnit = "rad") = 0, rel_angle_2(displayUnit = "rad") = 0, rel_angle_3(displayUnit = "rad") = 0, rel_pos_x = 1, rel_pos_y = 1, rel_pos_z = 1)  annotation(
          Placement(visible = true, transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles annotation(
          Placement(visible = true, transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
          Placement(visible = true, transformation(origin = {50, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.RelativePosition relativePosition annotation(
          Placement(visible = true, transformation(origin = {10, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.RelativeAngles relativeAngles annotation(
          Placement(visible = true, transformation(origin = {10, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation(angles = {20, 30, 40}, r = {10, 10, 10})  annotation(
          Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
  connect(relativePositionInit.frame_b, body.frame_a) annotation(
          Line(points = {{10, 10}, {68, 10}}, color = {95, 95, 95}));
  connect(fixed.frame_b, fixedRotation.frame_a) annotation(
          Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(fixedRotation.frame_b, relativePositionInit.frame_a) annotation(
          Line(points = {{-40, 10}, {-10, 10}}, color = {95, 95, 95}));
  connect(relativePosition.frame_a, fixedRotation.frame_b) annotation(
          Line(points = {{0, 60}, {-40, 60}, {-40, 10}}));
  connect(relativePosition.frame_b, body.frame_a) annotation(
          Line(points = {{20, 60}, {68, 60}, {68, 10}}, color = {95, 95, 95}));
  connect(relativeAngles.frame_a, fixedRotation.frame_b) annotation(
          Line(points = {{0, 88}, {-40, 88}, {-40, 10}}));
  connect(relativeAngles.frame_b, body.frame_a) annotation(
          Line(points = {{20, 88}, {68, 88}, {68, 10}}));
  connect(absoluteAngles.frame_a, relativePositionInit.frame_b) annotation(
          Line(points = {{40, -30}, {10, -30}, {10, 10}}, color = {95, 95, 95}));
  connect(absolutePosition.frame_a, relativePositionInit.frame_b) annotation(
          Line(points = {{40, -52}, {10, -52}, {10, 10}}, color = {95, 95, 95}));
      protected
  annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end RelativeInit;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Launchpad;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Components;

  model Clocked
    //Real int;
    Modelica.Blocks.Sources.Sine sine(f = 1) annotation(
      Placement(visible = true, transformation(origin = {-86, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = 10) annotation(
      Placement(visible = true, transformation(origin = {-88, -48}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    inner Modelica.Blocks.Noise.GlobalSeed globalSeed annotation(
      Placement(visible = true, transformation(origin = {-64, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Sensors.Internal.SampleClockedWithADeffects sample1(bias = 1, biased = true, limited = true, noisy = false, sigma = 0.3, yMax = 1.5) annotation(
      Placement(visible = true, transformation(origin = {-20, 10}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  equation
    connect(sine.y, sample1.u) annotation(
      Line(points = {{-74, 10}, {-27, 10}}, color = {0, 0, 127}));
    connect(periodicClock1.y, sample1.clock) annotation(
      Line(points = {{-82, -48}, {-20, -48}, {-20, 3}}, color = {175, 175, 175}));
//  int = interval(add.y);
  protected
    annotation(
      Icon(coordinateSystem(grid = {2, 0})),
      Diagram);
  end Clocked;

  model MyNoise
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    replaceable Modelica.Blocks.Noise.UniformNoise noise annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(noise.y, y) annotation(
      Line(points = {{9, 0}, {104, 0}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(grid = {2, 0}), graphics = {Text(origin = {0, -2}, extent = {{-88, 92}, {88, -92}}, textString = "N")}));
  end MyNoise;
end Tests;
