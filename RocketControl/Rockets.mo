within RocketControl;

package Rockets
  package Internal
    model PartialAirframe
    extends Components.Parts.LaunchPad.PartialLaunchMount;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a ref_center annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    equation

      annotation(
        Icon(graphics = {Text(origin = {467.111, 32}, lineColor = {128, 128, 128}, extent = {{-423.111, -41}, {-311.111, -82}}, textString = "ref_center")}));
    end PartialAirframe;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Internal;

  package Lynx

    package Aerodynamics
      model AerodynamicForce
        extends RocketControl.Aerodynamics.PartialAerodynamicForce;
        
        class AeroData = RocketControl.Aerodynamics.AeroData(redeclare type State = State);
        AeroData.ExternalAeroData aerodata = AeroData.ExternalAeroData();
        
        Real state_arr[State];
      equation
      
        state_arr[State.alpha] = Modelica.Units.Conversions.to_deg(aeroState.alpha);
        state_arr[State.beta] = Modelica.Units.Conversions.to_deg(aeroState.beta);
        state_arr[State.mach] = aeroState.mach;
        state_arr[State.altitude] = aeroState.altitude;
        
        coeffs = AeroData.getData(aerodata, state_arr);
       
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end AerodynamicForce;
    
      type State = enumeration(alpha, mach, beta, altitude);
    
      model Aerodynamics
        extends RocketControl.Aerodynamics.PartialAerodynamics(redeclare AerodynamicForce aerodynamicForce);
      equation
    
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Aerodynamics;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Aerodynamics;

    package AerodynamicsWithCanards
      model AerodynamicForce
        extends RocketControl.Aerodynamics.PartialAerodynamicForce;
        
        class AeroData = RocketControl.Aerodynamics.AeroData(redeclare type State = State);
        
        AeroData.ExternalAeroData aerodata = AeroData.ExternalAeroData(states_file, coeffs_file);
        
        Real state_arr[State];
    Modelica.Blocks.Interfaces.RealInput finDeflection[4] annotation(
          Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      protected
        constant String states_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_states_fins.npz");
        constant String coeffs_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_coeffs_fins.npz");
      equation
      
        state_arr[State.alpha] = Modelica.Units.Conversions.to_deg(aeroState.alpha);
        state_arr[State.beta] = Modelica.Units.Conversions.to_deg(aeroState.beta);
        state_arr[State.mach] = aeroState.mach;
        state_arr[State.altitude] = aeroState.altitude;
        state_arr[State.fin2delta1] = finDeflection[1];
        state_arr[State.fin2delta2] = finDeflection[2];
        state_arr[State.fin2delta3] = finDeflection[3];
        state_arr[State.fin2delta4] = finDeflection[4];
        
        coeffs = AeroData.getData(aerodata, state_arr);
       
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end AerodynamicForce;
    
      type State = enumeration(alpha, mach, beta, altitude, fin2delta1, fin2delta2, fin2delta3, fin2delta4);
    
      model Aerodynamics
        extends RocketControl.Aerodynamics.PartialAerodynamics(redeclare AerodynamicForce aerodynamicForce);
    Modelica.Blocks.Interfaces.RealInput finDeflection[4] annotation(
          Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
      connect(finDeflection, aerodynamicForce.finDeflection)
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Aerodynamics;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AerodynamicsWithCanards;
    model LynxAirframe
    extends RocketControl.Rockets.Internal.PartialAirframe;
    Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.06, I_21 = 0, I_22 = 6.436, I_31 = 0, I_32 = 0, I_33 = 6.437, enforceStates = false, m = 18.362, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
        Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(r = {-0.02, 0, -0.075}) annotation(
        Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(r = {-0.43, 0, -0.075}) annotation(
        Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    RocketControl.Components.Motors.M2000R m2000r(start_delay = 2) annotation(
        Placement(visible = true, transformation(origin = {30, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzleTranslation(r = {-1.150, 0, 0}) annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    equation
      connect(ref_center, rocket.frame_a) annotation(
        Line(points = {{100, 0}, {30, 0}, {30, 60}}));
      connect(lug_bow.frame_a, rocket.frame_a) annotation(
        Line(points = {{-40, 40}, {30, 40}, {30, 60}}, color = {95, 95, 95}));
      connect(lug_bow.frame_b, frame_a) annotation(
        Line(points = {{-60, 40}, {-74, 40}, {-74, 60}, {-100, 60}}, color = {95, 95, 95}));
      connect(frame_a1, lug_aft.frame_b) annotation(
        Line(points = {{-100, -40}, {-60, -40}}));
      connect(lug_aft.frame_a, rocket.frame_a) annotation(
        Line(points = {{-40, -40}, {0, -40}, {0, 40}, {30, 40}, {30, 60}}));
      connect(m2000r.frame_b, nozzleTranslation.frame_a) annotation(
        Line(points = {{30, -54.2}, {30, -40.2}}, color = {95, 95, 95}));
    connect(nozzleTranslation.frame_b, rocket.frame_a) annotation(
        Line(points = {{30, -20}, {30, 60}}, color = {95, 95, 95}));
      annotation(
        Icon(graphics = {Polygon(lineColor = {60, 60, 61}, fillColor = {97, 183, 229}, fillPattern = FillPattern.VerticalCylinder, points = {{-20, 60}, {0, 100}, {20, 60}, {20, -60}, {40, -100}, {-40, -100}, {-20, -60}, {-20, 60}}), Text(origin = {-6, 16},lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name")}));
    end LynxAirframe;

    model Sensors
     extends Modelica.Icons.RoundSensor;
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
  RocketControl.Components.Sensors.RealMagnetometer realMagnetometer(b_max = 2500e3, bias = 350, bits = 16, fixedLocalSeed = {99, 432, 543543}, misalignement = {0.2, -0.1, 0.3}, samplePeriodMs = 20, sigmaNoise = 10) annotation(
        Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealAccelerometer idealAccelerometer annotation(
        Placement(visible = true, transformation(origin = {-10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles(sequence = {3, 2, 1}) annotation(
        Placement(visible = true, transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealMagnetometer idealMagnetometer annotation(
        Placement(visible = true, transformation(origin = {-10, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealGyroscope realGyroscope(bias = {0.1, -0.05, -0.02}, bits = 16, fixedLocalSeed = {9, 423, 43214321}, rate_max = 250, samplePeriodMs = 20, sigmaARW = 0.5, sigmaRRW = 1) annotation(
        Placement(visible = true, transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealAccelerometer realAccelerometer(acc_max = 156.96, bias = {0.02, 0.04, -0.03}, bits = 16, fixedLocalSeed = {637914168, 1062993719, 2034216499}, samplePeriodMs = 20, sigmaBiasInstability = 0.01, sigmaNoise = 0.1) annotation(
        Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealGyroscope idealGyroscope annotation(
        Placement(visible = true, transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Sensors.IdealBarometer idealBarometer annotation(
        Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealBarometer realBarometer(bias(displayUnit = "Pa") = 300, bits = 24, fixedLocalSeed = 5325587, p_max(displayUnit = "Pa") = 110000, p_min (displayUnit = "Pa") = 1000, samplePeriodMs = 50, sigmaNoise(displayUnit = "Pa") = 50)  annotation(
        Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)  annotation(
        Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealGNSS realGNSS(fixedLocalSeed = {1775435783, 568478634, -1728550798},samplePeriodMs = 20, sigmaNoise_xy = 5, sigmaNoise_z = 10)  annotation(
        Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion quat_true annotation(
        Placement(visible = true, transformation(origin = {28, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler eul_true annotation(
        Placement(visible = true, transformation(origin = {68, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Math.Blocks.UnwrapAngle unwrapAngle(internalClock = true, n = 3)  annotation(
        Placement(visible = true, transformation(origin = {116, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Interfaces.AvionicsBus bus annotation(
        Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.AssetSensor assetSensor annotation(
        Placement(visible = true, transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
    
      connect(frame_a, realMagnetometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, 50}, {-20, 50}}));
      connect(frame_a, realGyroscope.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, 70}, {-20, 70}}));
      connect(frame_a, realAccelerometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, 90}, {-20, 90}}));
      connect(frame_a, absoluteAngles.frame_a) annotation(
        Line(points = {{-100, 0}, {-90, 0}, {-90, 30}, {-80, 30}}));
      connect(frame_a, idealAccelerometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -50}, {-20, -50}}));
      connect(frame_a, idealGyroscope.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -70}, {-20, -70}}));
      connect(frame_a, idealMagnetometer.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}, {-40, -90}, {-20, -90}}));
  connect(realAccelerometer.acc, bus.a_meas) annotation(
        Line(points = {{0, 90}, {20.5, 90}, {20.5, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realGyroscope.w_meas, bus.w_meas) annotation(
        Line(points = {{0, 70}, {20, 70}, {20, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realMagnetometer.b_meas, bus.b_meas) annotation(
        Line(points = {{0, 50}, {19, 50}, {19, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealBarometer.frame_a, frame_a) annotation(
        Line(points = {{-20, -30}, {-40, -30}, {-40, 0}, {-100, 0}}, color = {95, 95, 95}));
      connect(realBarometer.frame_a, frame_a) annotation(
        Line(points = {{-20, 30}, {-40, 30}, {-40, 0}, {-100, 0}}));
  connect(realBarometer.press, bus.p_meas) annotation(
        Line(points = {{0, 30}, {20, 30}, {20, 100}, {100, 100}}, color = {0, 0, 127}));
      connect(frame_a, absolutePosition.frame_a) annotation(
        Line(points = {{-100, 0}, {-90, 0}, {-90, -70}, {-80, -70}}));
      connect(realGNSS.frame_a, frame_a) annotation(
        Line(points = {{-20, 10}, {-59, 10}, {-59, 0}, {-100, 0}}, color = {95, 95, 95}));
      connect(realGNSS.pos, bus.x_meas) annotation(
        Line(points = {{0, 10}, {20, 10}, {20, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      connect(frame_a, quat_true.frame_a) annotation(
        Line(points = {{-100, 0}, {18, 0}, {18, -2}}));
      connect(quat_true.q, eul_true.q) annotation(
        Line(points = {{38, -2}, {58, -2}}, color = {0, 0, 127}));
  connect(unwrapAngle.angle, eul_true.eul) annotation(
        Line(points = {{104, -2}, {78, -2}}, color = {0, 0, 127}));
  connect(frame_a, assetSensor.frame_a) annotation(
        Line(points = {{-100, 0}, {-90, 0}, {-90, -30}, {-80, -30}}));
    protected
      annotation(
        Icon(graphics = {Line(origin = {-83, 0}, points = {{-13, 0}, {13, 0}}), Text(origin = {-1.42109e-14, -40}, extent = {{-60, 20}, {60, -20}}, textString = "sens"), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name"), Line(origin = {85, 0}, points = {{-15, 0}, {9, 0}, {15, 0}})}));
    end Sensors;

    model Estimators
    extends RocketControl.GNC.Internal.Icons.Navigation;
    
      RocketControl.GNC.Navigation.AttitudeEstimation attitude_est (elevation0 = 84, heading0 = 130, samplingPeriodMs = 20, sigma_b = 2, sigma_u = from_deg(10), sigma_v = from_deg(60)) annotation(
        Placement(visible = true, transformation(origin = {-52, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler eul_est annotation(
        Placement(visible = true, transformation(origin = {-12, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.UnwrapAngle unwrapAngle(n = 3)  annotation(
        Placement(visible = true, transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Interfaces.AvionicsBus bus annotation(
        Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
  connect(bus.w_meas, attitude_est.w_meas_degs) annotation(
        Line(points = {{100, 100}, {18, 100}, {18, 60}, {-64, 60}}, thickness = 0.5));
  connect(bus.b_meas, attitude_est.b_meas_nt) annotation(
        Line(points = {{100, 100}, {18, 100}, {18, 52}, {-64, 52}}, thickness = 0.5));
  connect(bus.x_meas, attitude_est.r_0_est) annotation(
        Line(points = {{100, 100}, {16, 100}, {16, 44}, {-64, 44}}, thickness = 0.5));
  connect(attitude_est.q_est, bus.q_est) annotation(
        Line(points = {{-41, 57}, {-30, 57}, {-30, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
  connect(attitude_est.w_est_degs, bus.w_est) annotation(
        Line(points = {{-41, 47}, {-30, 47}, {-30, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
  connect(attitude_est.q_est, eul_est.q) annotation(
        Line(points = {{-41, 57}, {-30, 57}, {-30, 10}, {-24, 10}}, color = {0, 0, 127}));
  connect(eul_est.eul, unwrapAngle.angle) annotation(
        Line(points = {{-1, 10}, {18, 10}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Line(origin = {92, 0}, points = {{-2, 0}, {2, 0}})}),
        Diagram);
    end Estimators;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Lynx;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Rockets;
