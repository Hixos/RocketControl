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

      model SensorTestSuite
      RocketControl.World.MyWorld myWorld(n = {0, 0, 1})  annotation(
          Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_r = true, use_v = true, use_w = true, v_rel_a_1(fixed = true), v_rel_a_2(fixed = true, start = 20), v_rel_a_3(fixed = true, start = -400), w_rel_b_1(fixed = true, start = 1), w_rel_b_2(fixed = true, start = 2), w_rel_b_3(fixed = true, start = 3))  annotation(
          Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealGyroscope realGyroscope(bias(each displayUnit = "rad/s") = {1, 2, 3}, biased = true, bits = 14, limited = true, noisy = true, quantized = true, rate_max(displayUnit = "rad/s") = 10, samplePeriodMs = 20, sigmaARW(displayUnit = "rad/s") = 1, sigmaRRW(displayUnit = "rad/s2") = 0.1)  annotation(
          Placement(visible = true, transformation(origin = {30, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealAccelerometer realAccelerometer(acc_max = 140, bias = {1, 2, 3}, biased = true, limited = true, noisy = true, quantized = true, samplePeriodMs = 20, sigmaBiasInstability = 0.1, sigmaNoise = 1)  annotation(
          Placement(visible = true, transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealMagnetometer realMagnetometer(b_max(displayUnit = "nT") = 0.0001000000000000001, bias(displayUnit = "nT") = 3.000000000000003e-07, biased = true, bits = 16, limited = true, misalignement(each displayUnit = "rad") = {0.1, 0.2, 0.3}, noisy = true, quantized = true, samplePeriodMs = 20, sigmaNoise (displayUnit = "T") = 3.000000000000002e-08)  annotation(
          Placement(visible = true, transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealBarometer realBarometer(bias(displayUnit = "Pa") = 323, biased = true, bits = 12, limited = true, noisy = true, p_max(displayUnit = "Pa") = 100000, quantized = true, samplePeriodMs = 20, sigmaNoise(displayUnit = "Pa") = 10)  annotation(
          Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealGNSS realGNSS(noisy = true, samplePeriodMs = 20, sigmaNoise_vxy = 0, sigmaNoise_vz = 0, sigmaNoise_xy = 0, sigmaNoise_z = 0, sin_error_freq = 0) annotation(
          Placement(visible = true, transformation(origin = {34, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(freeMotionScalarInit.frame_a, myWorld.frame_b) annotation(
          Line(points = {{-80, 0}, {-80, -90}}));
        connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
          Line(points = {{-60, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
        connect(realGyroscope.frame_a, body.frame_a) annotation(
          Line(points = {{20, 88}, {-8, 88}, {-8, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
        connect(realAccelerometer.frame_a, body.frame_a) annotation(
          Line(points = {{20, 50}, {-10, 50}, {-10, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
        connect(realMagnetometer.frame_a, body.frame_a) annotation(
          Line(points = {{20, 10}, {-8, 10}, {-8, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
        connect(realBarometer.frame_a, body.frame_a) annotation(
          Line(points = {{20, -30}, {-10, -30}, {-10, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
  connect( body.frame_a,realGNSS.frame_a) annotation(
          Line(points = {{-30, 20}, {-30, 0}, {-8, 0}, {-8, -68}, {24, -68}}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end SensorTestSuite;

      model ADEffect
      RocketControl.Components.Sensors.Internal.ADeffects aDeffects(biased = true, noisy = false, samplePeriodMs = 20)  annotation(
          Placement(visible = true, transformation(origin = {6, 10}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine(amplitude = 10, f = 1)  annotation(
          Placement(visible = true, transformation(origin = {-70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(sine.y, aDeffects.u) annotation(
          Line(points = {{-59, 10}, {-1, 10}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end ADEffect;
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

      model RampTest
      RocketControl.GNC.Control.Ramp ramp annotation(
          Placement(visible = true, transformation(origin = {-2, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 10)  annotation(
          Placement(visible = true, transformation(origin = {-74, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(const.y, ramp.u) annotation(
          Line(points = {{-62, -4}, {-12, -4}, {-12, -8}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end RampTest;

      model ActuatorLQ
        RocketControl.GNC.Control.LinearLQ.RocketAndActuator systemMatrices(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22, wa = 13) annotation(
          Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, maxiter = 15, n = 9, tol = 1e-40, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({1, 1, 0.4} * 20)) annotation(
          Placement(visible = true, transformation(origin = {-30, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 9, val = diagonal({0, 10, 10, 100, 0, 0, 0, 0, 0})) annotation(
          Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant annotation(
          Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant vel(k = {200, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant ang_vel(k = {0, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {-90, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant x_est(k = {0, 0, -1000})  annotation(
          Placement(visible = true, transformation(origin = {-92, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant control(k = {1, 0, 0})  annotation(
          Placement(visible = true, transformation(origin = {-92, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate annotation(
          Placement(visible = true, transformation(origin = {-48, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 6)  annotation(
          Placement(visible = true, transformation(origin = {2, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant1(k = {10, 0}, n = 2) annotation(
          Placement(visible = true, transformation(origin = {-116, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.TransferFunction transferFunction(a = {0.07692, 1}, b = {0, 1}, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
          Placement(visible = true, transformation(origin = {85, -33}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0}, n = 2) annotation(
          Placement(visible = true, transformation(origin = {-92, -116}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(systemMatrices.A, continuousLQR.A) annotation(
          Line(points = {{-19, 35}, {10, 35}, {10, -2}, {18, -2}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{-19, -42}, {6, -42}, {6, -14}, {18, -14}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{-19, -10}, {18, -10}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.B, continuousLQR.B) annotation(
          Line(points = {{-19, 25}, {6, 25}, {6, -6}, {18, -6}}, color = {0, 0, 127}, thickness = 0.5));
        connect(booleanConstant.y, continuousLQR.enable) annotation(
          Line(points = {{2, 90}, {30, 90}, {30, 0}}, color = {255, 0, 255}));
        connect(vel.v, systemMatrices.vel) annotation(
          Line(points = {{-79, 70}, {-58, 70}, {-58, 40}, {-42, 40}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ang_vel.v, systemMatrices.ang_vel) annotation(
          Line(points = {{-78, 34}, {-42, 34}}, color = {0, 0, 127}, thickness = 0.5));
        connect(x_est.v, systemMatrices.x_est) annotation(
          Line(points = {{-80, 4}, {-62, 4}, {-62, 28}, {-42, 28}}, color = {0, 0, 127}, thickness = 0.5));
        connect(control.v, systemMatrices.control_cmd) annotation(
          Line(points = {{-80, -28}, {-56, -28}, {-56, 22}, {-42, 22}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ang_vel.v, vectorConcatenate.v2) annotation(
          Line(points = {{-78, 34}, {-72, 34}, {-72, -78}, {-60, -78}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-36, -74}, {-10, -74}, {-10, -76}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{14, -80}, {40, -80}, {40, -44}, {12, -44}, {12, -18}, {18, -18}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vel.v[1], vectorConcatenate.v1[1]) annotation(
          Line(points = {{-78, 70}, {-72, 70}, {-72, -70}, {-60, -70}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConstant1.v[1], vectorConcatenate.v1[2]) annotation(
          Line(points = {{-104, -64}, {-64, -64}, {-64, -70}, {-60, -70}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConstant1.v[2], vectorConcatenate.v1[3]) annotation(
          Line(points = {{-104, -64}, {-60, -64}, {-60, -70}}, color = {0, 0, 127}, thickness = 0.5));
  connect(continuousLQR.u[1], transferFunction.u) annotation(
          Line(points = {{42, -10}, {62, -10}, {62, -32}, {80, -32}}, color = {0, 0, 127}));
  connect(transferFunction.y, vectorConcatenate1.v2[1]) annotation(
          Line(points = {{90, -32}, {92, -32}, {92, -98}, {-24, -98}, {-24, -84}, {-10, -84}}, color = {0, 0, 127}));
  connect(vectorConstant.v[2], vectorConcatenate1.v2[2]) annotation(
          Line(points = {{-80, -116}, {-10, -116}, {-10, -84}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConstant.v[1], vectorConcatenate1.v2[3]) annotation(
          Line(points = {{-80, -116}, {-10, -116}, {-10, -84}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end ActuatorLQ;
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
end Tests;
