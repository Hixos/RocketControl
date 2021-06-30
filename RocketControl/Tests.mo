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
    RocketControl.Aerodynamics.AerodynamicForce aerodynamicForce annotation(
      Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
      Line(points = {{14, 0}, {40, 0}}));
    connect(aerodynamicForce.frame_b, fixed.frame_b) annotation(
      Line(points = {{60, 0}, {80, 0}}, color = {95, 95, 95}));
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

  model AeroDataTest
    import RocketControl.Aerodynamics.*;
    AeroData aerodata = AeroData();
    SI.Angle alpha;
    Real state[State];
    Real coeff[Coeff];
  initial equation
    alpha = Modelica.Units.Conversions.from_deg(-50);
  equation
    der(alpha) = 0.17;
    state[State.alphas] = alpha;
    state[State.betas] = 0.1;
    state[State.machs] = 0.5;
    state[State.alts] = 500;
  algorithm
    coeff := coeffValue(aerodata, state);
  end AeroDataTest;

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

  model FMUTest
    import Modelica.Units.Conversions.from_deg;
    parameter SI.Angle elevation(displayUnit = "deg") = from_deg(87);
    parameter SI.Angle azimuth(displayUnit = "deg") = from_deg(45);
    parameter SI.AngularVelocity w0[3] = from_deg({0, 0, 0});
    parameter SI.Velocity v0[3] = {0, 0, -100};
    Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.066590312, I_21 = -0.003190759, I_22 = 9.822815884, I_31 = 0.00128563, I_32 = -0.000234088, I_33 = 9.822815884, angles_fixed = true, angles_start(each displayUnit = "rad") = {azimuth, elevation, 0}, enforceStates = true, m = 22, r_0(each fixed = true), r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, sequence_start = {3, 2, 1}, useQuaternions = true, v_0(each fixed = true, start = v0), w_0_fixed = false, w_a(each fixed = true, start = w0)) annotation(
      Placement(visible = true, transformation(origin = {-36, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    inner RocketControl.World.Atmosphere atmosphere annotation(
      Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner RocketControl.World.MyWorld world(n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(sequence = {3, 2, 1}) annotation(
      Placement(visible = true, transformation(origin = {32, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput asset[3] annotation(
      Placement(visible = true, transformation(origin = {92, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {96, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput r[3] annotation(
      Placement(visible = true, transformation(origin = {92, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {84, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
      Placement(visible = true, transformation(origin = {32, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Aerodynamics.Aerodynamics aerodynamics annotation(
      Placement(visible = true, transformation(origin = {70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Sensors.AeroStateSensor aeroStateSensor annotation(
      Placement(visible = true, transformation(origin = {24, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Interfaces.AeroStateDemux aeroStateDemux annotation(
      Placement(visible = true, transformation(origin = {50, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput alpha annotation(
      Placement(visible = true, transformation(origin = {92, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {84, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput beta annotation(
      Placement(visible = true, transformation(origin = {92, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {84, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput altitude annotation(
      Placement(visible = true, transformation(origin = {92, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {84, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput mach annotation(
      Placement(visible = true, transformation(origin = {92, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {84, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput v[3] annotation(
      Placement(visible = true, transformation(origin = {92, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {84, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput w[3] annotation(
      Placement(visible = true, transformation(origin = {92, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {84, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzleTranslation(r = {-1.150, 0, 0}) annotation(
      Placement(visible = true, transformation(origin = {-36, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  RocketControl.Components.Motors.M2000RNOTT m2000rnott annotation(
      Placement(visible = true, transformation(origin = {-36, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    when rocket.r_0[3] > 0 and time > 1 then
      terminate("Simulation terminated successfully");
    end when;
    connect(absoluteAngles.frame_a, rocket.frame_a) annotation(
      Line(points = {{22, 20}, {-36, 20}, {-36, 60}}, color = {95, 95, 95}));
    connect(absoluteAngles.angles, asset) annotation(
      Line(points = {{43, 20}, {91, 20}}, color = {0, 0, 127}));
    connect(absolutePosition.frame_a, rocket.frame_a) annotation(
      Line(points = {{22, -6}, {4, -6}, {4, 20}, {-36, 20}, {-36, 60}}));
    connect(absolutePosition.r, r) annotation(
      Line(points = {{44, -6}, {92, -6}}, color = {0, 0, 127}));
    connect(aerodynamics.frame_b, rocket.frame_a) annotation(
      Line(points = {{60, 60}, {14, 60}, {14, 34}, {-36, 34}, {-36, 60}}, color = {95, 95, 95}));
    connect(aeroStateSensor.frame_a, rocket.frame_a) annotation(
      Line(points = {{14, -44}, {4, -44}, {4, 22}, {-36, 22}, {-36, 60}}, color = {95, 95, 95}));
    connect(aeroStateDemux.state, aeroStateSensor.aeroStateOutput) annotation(
      Line(points = {{40, -44}, {34, -44}}));
    connect(aeroStateDemux.alpha, alpha) annotation(
      Line(points = {{60, -34}, {70, -34}, {70, -24}, {92, -24}}, color = {0, 0, 127}));
    connect(aeroStateDemux.beta, beta) annotation(
      Line(points = {{60, -38}, {74, -38}, {74, -36}, {92, -36}}, color = {0, 0, 127}));
    connect(aeroStateDemux.altitude, altitude) annotation(
      Line(points = {{60, -42}, {74, -42}, {74, -48}, {92, -48}}, color = {0, 0, 127}));
    connect(aeroStateDemux.mach, mach) annotation(
      Line(points = {{60, -46}, {72, -46}, {72, -60}, {92, -60}}, color = {0, 0, 127}));
    connect(aeroStateDemux.v, v) annotation(
      Line(points = {{60, -50}, {68, -50}, {68, -72}, {92, -72}}, color = {0, 0, 127}, thickness = 0.5));
    connect(aeroStateDemux.w, w) annotation(
      Line(points = {{60, -52}, {66, -52}, {66, -84}, {92, -84}}, color = {0, 0, 127}, thickness = 0.5));
    connect(nozzleTranslation.frame_b, rocket.frame_a) annotation(
      Line(points = {{-36, -14}, {-36, 60}}, color = {95, 95, 95}));
  connect(nozzleTranslation.frame_a, m2000rnott.frame_b) annotation(
      Line(points = {{-36, -34}, {-36, -64}}));
  protected
  end FMUTest;

  model LaunchRailTest
  
   import Modelica.Units.Conversions.from_deg;
   parameter SI.Mass m = 1;
   parameter SI.Distance s_max = 0.001;
   parameter SI.ModulusOfElasticity c_x = Modelica.Constants.g_n*m/s_max;
   parameter SI.ModulusOfElasticity c_y = Modelica.Constants.g_n*m/s_max;
   parameter SI.ModulusOfElasticity c_z = Modelica.Constants.g_n*m/s_max;
   
   parameter SI.ModulusOfElasticity d_x = 2*sqrt(c_x*m)*1.01;
   parameter SI.ModulusOfElasticity d_y = 2*sqrt(c_y*m)*1.01;
   parameter SI.ModulusOfElasticity d_z = 2*sqrt(c_z*m)*1.01;
  Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 1, I_22 = 1, I_33 = 1,enforceStates = true, m = m, r_CM = {0, 0, 0}) annotation(
      Placement(visible = true, transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1})  annotation(
      Placement(visible = true, transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.UniformGravity, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(sequence = {3, 2, 1})  annotation(
      Placement(visible = true, transformation(origin = {70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b)  annotation(
      Placement(visible = true, transformation(origin = {40, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
      Placement(visible = true, transformation(origin = {90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Step step(height = m * 9.81 * 4, startTime = 0.2)  annotation(
      Placement(visible = true, transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(
      Placement(visible = true, transformation(origin = {70, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation base_bow(r = {1, 0, -0.075}) annotation(
      Placement(visible = true, transformation(origin = {12, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation base_aft(r = {-1, 0, -0.075}) annotation(
      Placement(visible = true, transformation(origin = {12, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 0, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation (displayUnit = "deg") = 75, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4)  annotation(
      Placement(visible = true, transformation(origin = {-48, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzle(r = {-1, 0, 0}) annotation(
      Placement(visible = true, transformation(origin = {40, -12}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce worldForce(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
      Placement(visible = true, transformation(origin = {-10, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-84, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Step step1(height = -1, offset = 1, startTime = 1) annotation(
      Placement(visible = true, transformation(origin = {-84, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
  connect(absoluteAngles.frame_a, body.frame_a) annotation(
      Line(points = {{60, 60}, {40, 60}, {40, 30}}, color = {95, 95, 95}));
  connect(const.y, force.force[2]) annotation(
      Line(points = {{79, -50}, {50.5, -50}, {50.5, -68}, {40, -68}}, color = {0, 0, 127}));
  connect(const.y, force.force[3]) annotation(
      Line(points = {{79, -50}, {50.5, -50}, {50.5, -68}, {40, -68}}, color = {0, 0, 127}));
  connect(step.y, force.force[1]) annotation(
      Line(points = {{79, -90}, {40, -90}, {40, -68}}, color = {0, 0, 127}));
  connect(absoluteAngularVelocity.frame_a, body.frame_a) annotation(
      Line(points = {{60, 90}, {40, 90}, {40, 30}}, color = {95, 95, 95}));
  connect(base_bow.frame_a, body.frame_a) annotation(
      Line(points = {{22, 50}, {40, 50}, {40, 30}}, color = {95, 95, 95}));
  connect(base_aft.frame_a, body.frame_a) annotation(
      Line(points = {{22, 10}, {31, 10}, {31, 30}, {40, 30}}));
  connect(launchRail.frame_a, fixed.frame_b) annotation(
      Line(points = {{-58, 30}, {-74, 30}, {-74, -50}, {-80, -50}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_aft, base_aft.frame_b) annotation(
      Line(points = {{-38, 24}, {-18, 24}, {-18, 10}, {2, 10}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, base_bow.frame_b) annotation(
      Line(points = {{-38, 36}, {-18, 36}, {-18, 50}, {2, 50}}, color = {95, 95, 95}));
  connect(nozzle.frame_a, body.frame_a) annotation(
      Line(points = {{40, -2}, {40, 30}}));
  connect(force.frame_b, nozzle.frame_b) annotation(
      Line(points = {{40, -46}, {40, -22}}));
  connect(worldForce.frame_b, base_bow.frame_b) annotation(
      Line(points = {{0, 80}, {2, 80}, {2, 50}}, color = {95, 95, 95}));
  connect(constant1.y, worldForce.force[1]) annotation(
      Line(points = {{-72, 84}, {-22, 84}, {-22, 80}}, color = {0, 0, 127}));
  connect(constant1.y, worldForce.force[2]) annotation(
      Line(points = {{-72, 84}, {-22, 84}, {-22, 80}}, color = {0, 0, 127}));
  connect(step1.y, worldForce.force[3]) annotation(
      Line(points = {{-72, 50}, {-22, 50}, {-22, 80}}, color = {0, 0, 127}));
  end LaunchRailTest;
end Tests;
