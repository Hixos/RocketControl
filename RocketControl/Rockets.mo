within RocketControl;

package Rockets
extends Internal.Icon;

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
        connect(finDeflection, aerodynamicForce.finDeflection) annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Aerodynamics;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AerodynamicsWithCanards;

    model LynxBody
    parameter SI.Time start_delay = 0.5;
    extends Rockets.Internal.PartialRocketBody;
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(r = {-0.02, 0, -0.075}) annotation(
        Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(r = {-0.43, 0, -0.075}) annotation(
        Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzleTranslation(r = {-1.150, 0, 0}) annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.06, I_21 = 0, I_22 = 6.436, I_31 = 0, I_32 = 0, I_33 = 6.437, enforceStates = false, m = 22, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
        Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  RocketControl.Components.Propulsion.M2000R m2000r(start_delay = start_delay)  annotation(
        Placement(visible = true, transformation(origin = {30, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(rocket.frame_a, nozzleTranslation.frame_b) annotation(
        Line(points = {{30, 60}, {30, -20}}));
  connect(rocket.frame_a, ref_center) annotation(
        Line(points = {{30, 60}, {30, 0}, {100, 0}}));
  connect(lug_bow.frame_a, rocket.frame_a) annotation(
        Line(points = {{-40, 40}, {0, 40}, {0, 22}, {30, 22}, {30, 60}}, color = {95, 95, 95}));
  connect(lug_aft.frame_a, rocket.frame_a) annotation(
        Line(points = {{-40, -40}, {0, -40}, {0, 22}, {30, 22}, {30, 60}}, color = {95, 95, 95}));
  connect(frame_lug_bow, lug_bow.frame_b) annotation(
        Line(points = {{-100, 60}, {-68, 60}, {-68, 40}, {-60, 40}}));
  connect(lug_aft.frame_b, frame_lug_aft) annotation(
        Line(points = {{-60, -40}, {-100, -40}}, color = {95, 95, 95}));
  connect(nozzleTranslation.frame_a, m2000r.frame_b) annotation(
        Line(points = {{30, -40}, {30, -64}}, color = {95, 95, 95}));
      annotation(
        Icon(graphics = {Text(origin = {0, -220}, lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name")}),
  Diagram);
    end LynxBody;

    model LynxRocket
    extends Rockets.Internal.PartialRocket;
  RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
        Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Aerodynamics.Aerodynamics aerodynamics annotation(
        Placement(visible = true, transformation(origin = {12, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
        Placement(visible = true, transformation(origin = {100, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
        Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
      connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
        Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
      connect(lynxBody.ref_center, ref_center) annotation(
        Line(points = {{-40, 0}, {100, 0}}));
  connect(aerodynamics.frame_b, lynxBody.ref_center) annotation(
        Line(points = {{2, 46}, {-12, 46}, {-12, 0}, {-40, 0}}, color = {95, 95, 95}));
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end LynxRocket;
    
    model LynxWithCanardsRocket
    extends Rockets.Internal.PartialRocket;
    RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
        Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
        Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
        Placement(visible = true, transformation(origin = {100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Actuators.ServoMotor servoMotor(a = {0.07692, 1}, b = {0, 1}, nservos = 4) annotation(
        Placement(visible = true, transformation(origin = {62, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    equation
      connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
        Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
      connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
        Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
      connect(lynxBody.ref_center, ref_center) annotation(
        Line(points = {{-40, 0}, {100, 0}}));
      connect(aerodynamics.frame_b, lynxBody.ref_center) annotation(
        Line(points = {{-20, 50}, {-30, 50}, {-30, 0}, {-40, 0}}, color = {95, 95, 95}));
  connect(bus.control_cmd, servoMotor.setpoint) annotation(
        Line(points = {{100, 90}, {100, 34}, {74, 34}}, thickness = 0.5));
  connect(servoMotor.servo_pos, aerodynamics.finDeflection) annotation(
        Line(points = {{52, 34}, {-26, 34}, {-26, 44}, {-20, 44}}, color = {0, 0, 127}, thickness = 0.5));
  connect(servoMotor.servo_pos, bus.fin_position) annotation(
        Line(points = {{52, 34}, {32, 34}, {32, 90}, {100, 90}}, color = {0, 0, 127}, thickness = 0.5));
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end LynxWithCanardsRocket;

    package GNC
    extends RocketControl.GNC.Internal.Icons.Navigation;
      package Navigation
      extends RocketControl.GNC.Internal.Icons.Navigation;

        model LynxIdealNavigation
        extends Rockets.Internal.PartialNavigationSystem;
  RocketControl.Interfaces.AvionicsBus bus annotation(
            Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Quaternion2Euler quaternion2Euler annotation(
            Placement(visible = true, transformation(origin = {10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.FlightPathBlocks.Track track annotation(
            Placement(visible = true, transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.FlightPathBlocks.ClimbAngle climbAngle annotation(
            Placement(visible = true, transformation(origin = {10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.FlightPathBlocks.Downrange downrange annotation(
            Placement(visible = true, transformation(origin = {10, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        protected
        Modelica.Blocks.Interfaces.RealInput v_est[3] annotation(
                  Placement(visible = true, transformation(extent = {{-8, 72}, {8, 88}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput x_est[3] annotation(
            Placement(visible = true, transformation(extent = {{-8, 52}, {8, 68}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput w_est[3] annotation(
            Placement(visible = true, transformation(extent = {{-8, 32}, {8, 48}}, rotation = 0)));
        equation
          connect(bus.v_meas, v_est) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 80}, {0, 80}}, thickness = 0.5));
  connect(v_est, bus.v_est) annotation(
            Line(points = {{0, 80}, {60, 80}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.x_meas, x_est) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 60}, {0, 60}}, thickness = 0.5));
  connect(x_est, bus.x_est) annotation(
            Line(points = {{0, 60}, {60, 60}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.w_meas, w_est) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 40}, {0, 40}}, thickness = 0.5));
  connect(w_est, bus.w_est) annotation(
            Line(points = {{0, 40}, {60, 40}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.v_est, track.v) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -20}, {-2, -20}}, thickness = 0.5));
  connect(bus.q_est, quaternion2Euler.q) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 10}, {-2, 10}}, thickness = 0.5));
  connect(track.v, climbAngle.v) annotation(
            Line(points = {{-2, -20}, {-14, -20}, {-14, -50}, {-2, -50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(climbAngle.v, downrange.x) annotation(
            Line(points = {{-2, -50}, {-14, -50}, {-14, -80}, {-2, -80}}, color = {0, 0, 127}, thickness = 0.5));
          annotation(
            Icon(coordinateSystem(grid = {2, 0})));
        end LynxIdealNavigation;

        model LynxNavigation
        outer World.SimOptions opt;
        extends Rockets.Internal.PartialNavigationSystem;
        RocketControl.Interfaces.AvionicsBus bus annotation(
            Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Navigation.AttitudeEstimation attitudeEstimation(elevation0 = 1.466076571675237, heading0 = 2.268928027592628, samplingPeriodMs = opt.samplePeriodMs, sigma_b = 2, sigma_u = from_deg(10), sigma_v = from_deg(60)) annotation(
            Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            
            RocketControl.GNC.Navigation.PositionEstimation positionEstimation(samplingPeriodMs = samplePeriodMs, sigma_gps = {1000, 1000, 1000, 100, 100, 100}, sigma_pos = 2, sigma_vel = 1) annotation(
            Placement(visible = true, transformation(origin = {-48, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.FlightPathBlocks.Downrange downrange annotation(
            Placement(visible = true, transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.FlightPathBlocks.Track track annotation(
            Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Quaternion2Euler quaternion2Euler annotation(
            Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.FlightPathBlocks.ClimbAngle climbAngle annotation(
            Placement(visible = true, transformation(origin = {30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            
        equation
  connect(bus.w_meas, attitudeEstimation.w_meas) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 58}, {-62, 58}}, thickness = 0.5));
  connect(bus.b_meas, attitudeEstimation.b_meas) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 50}, {-62, 50}}, thickness = 0.5));
  connect(attitudeEstimation.q_est, bus.q_est) annotation(
            Line(points = {{-39, 55}, {60, 55}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(attitudeEstimation.w_est, bus.w_est) annotation(
            Line(points = {{-39, 45}, {60, 45}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.q_est, positionEstimation.q) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 20}, {-60, 20}}, thickness = 0.5));
  connect(bus.a_meas, positionEstimation.acc_body) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 14}, {-60, 14}}, thickness = 0.5));
  connect(bus.x_meas, positionEstimation.pos_ned) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 6}, {-60, 6}}, thickness = 0.5));
  connect(bus.v_meas, positionEstimation.vel_ned) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 0}, {-60, 0}}, thickness = 0.5));
  connect(bus.x_est, attitudeEstimation.r_0_est) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 42}, {-62, 42}}, thickness = 0.5));
  connect(positionEstimation.pos_est, bus.x_est) annotation(
            Line(points = {{-36, 14}, {-14, 14}, {-14, 34}, {60, 34}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(positionEstimation.vel_est, bus.v_est) annotation(
            Line(points = {{-36, 6}, {-8, 6}, {-8, 26}, {60, 26}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.q_est, quaternion2Euler.q) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -16}, {0, -16}, {0, 0}, {18, 0}}, thickness = 0.5));
  connect(bus.v_est, track.v) annotation(
            Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -30}, {18, -30}}, thickness = 0.5));
  connect(track.v, climbAngle.v) annotation(
            Line(points = {{18, -30}, {0, -30}, {0, -60}, {18, -60}}, color = {0, 0, 127}, thickness = 0.5));
  connect(climbAngle.v, downrange.x) annotation(
            Line(points = {{18, -60}, {0, -60}, {0, -90}, {18, -90}}, color = {0, 0, 127}, thickness = 0.5));
          annotation(
            Icon(coordinateSystem(grid = {2, 0})));
        end LynxNavigation;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Navigation;

      package Sensors
      extends Modelica.Icons.SensorsPackage;
      model LynxIdealSensors
      extends Rockets.Internal.PartialSensorsPackage;
      RocketControl.Interfaces.AvionicsBus bus annotation(
            Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Aerodynamics.AeroStateSensor aeroStateSensor annotation(
            Placement(visible = true, transformation(origin = {-58, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealSensors.IdealGyroscope idealGyroscope annotation(
          Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealSensors.IdealAccelerometer idealAccelerometer annotation(
          Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealSensors.IdealMagnetometer idealMagnetometer annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealSensors.IdealBarometer idealBarometer annotation(
          Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealSensors.IdealGNSS idealGNSS annotation(
          Placement(visible = true, transformation(origin = {0, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealSensors.IdealAsset idealAsset annotation(
            Placement(visible = true, transformation(origin = {0, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      
      
      equation
        connect(frame_a, idealGyroscope.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 80}, {-10, 80}}));
      connect(frame_a, idealAccelerometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 40}, {-10, 40}}));
      connect(frame_a, idealMagnetometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-10, 0}}));
      connect(frame_a, idealBarometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, -40}, {-10, -40}}));
      connect(frame_a, idealGNSS.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, -80}, {-10, -80}}));
      connect(idealGyroscope.w, bus.w_meas) annotation(
          Line(points = {{10, 80}, {60, 80}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealAccelerometer.a, bus.a_meas) annotation(
          Line(points = {{10, 40}, {60, 40}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealMagnetometer.b, bus.b_meas) annotation(
          Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealBarometer.p, bus.p_meas) annotation(
          Line(points = {{10, -40}, {60, -40}, {60, 0}, {100, 0}}, color = {0, 0, 127}));
      connect(idealGNSS.x, bus.x_meas) annotation(
          Line(points = {{12, -76}, {60, -76}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(idealGNSS.v, bus.v_meas) annotation(
          Line(points = {{12, -84}, {60, -84}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(frame_a, idealAsset.frame_a) annotation(
            Line(points = {{-100, 0}, {-40, 0}, {-40, -18}, {-10, -18}}));
  connect(idealAsset.q, bus.q_est) annotation(
            Line(points = {{12, -18}, {60, -18}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(aeroStateSensor.frame_a, frame_a) annotation(
            Line(points = {{-68, 90}, {-80, 90}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end LynxIdealSensors;

        model LynxSampledIdealSensors
        outer World.SimOptions opt;
        
        extends Rockets.Internal.PartialSensorsPackage;
        RocketControl.Interfaces.AvionicsBus bus annotation(
              Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealGyroscope realGyroscope(bias(each displayUnit = "rad/s") = {0, 0, 0},fixedLocalSeed = {10, 100, 100}, noisy = true, samplePeriodMs = opt.samplePeriodMs, sigmaARW(displayUnit = "rad/s") = 0, sigmaRRW(displayUnit = "rad/s2") = 0)  annotation(
            Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealAccelerometer realAccelerometer(bias = {0, 0, 0}, fixedLocalSeed = {11, 101, 1001},noisy = true, samplePeriodMs = opt.samplePeriodMs, sigmaBiasInstability = 0, sigmaNoise = 0)  annotation(
            Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealMagnetometer realMagnetometer(b_max(displayUnit = "T") = 0, bias(each displayUnit = "T") = 0, fixedLocalSeed = {1211, 10211, 100211}, misalignement(each displayUnit = "rad") = {0, 0, 0}, noisy = true, samplePeriodMs = opt.samplePeriodMs, sigmaNoise(displayUnit = "T") = 0)  annotation(
            Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealBarometer realBarometer(bias(displayUnit = "Pa") = 0, fixedLocalSeed = 14, noisy = true, p_max(displayUnit = "Pa") = 0, samplePeriodMs = opt.samplePeriodMs, sigmaNoise(displayUnit = "Pa") = 0)  annotation(
            Placement(visible = true, transformation(origin = {0, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealGNSS realGNSS(fixedLocalSeed = {13, 103, 1003}, noisy = true, samplePeriodMs = opt.samplePeriodMs, sigmaNoise_vxy = 0, sigmaNoise_vz = 0, sigmaNoise_xy = 0, sigmaNoise_z = 0, sin_error_freq = 0)  annotation(
            Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
  connect(frame_a, realGyroscope.frame_a) annotation(
            Line(points = {{-100, 0}, {-40, 0}, {-40, 80}, {-10, 80}}));
  connect(frame_a, realAccelerometer.frame_a) annotation(
            Line(points = {{-100, 0}, {-40, 0}, {-40, 40}, {-10, 40}}));
  connect(frame_a, realMagnetometer.frame_a) annotation(
            Line(points = {{-100, 0}, {-10, 0}}));
  connect(frame_a, realGNSS.frame_a) annotation(
            Line(points = {{-100, 0}, {-40, 0}, {-40, -40}, {-10, -40}}));
  connect(frame_a, realBarometer.frame_a) annotation(
            Line(points = {{-100, 0}, {-40, 0}, {-40, -80}, {-10, -80}}));
  connect(realGyroscope.w, bus.w_meas) annotation(
            Line(points = {{11, 80}, {60, 80}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realAccelerometer.a, bus.a_meas) annotation(
            Line(points = {{10, 40}, {60, 40}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realMagnetometer.b, bus.b_meas) annotation(
            Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realGNSS.x, bus.x_meas) annotation(
            Line(points = {{12, -36}, {60, -36}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realGNSS.v, bus.v_meas) annotation(
            Line(points = {{12, -44}, {60, -44}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realBarometer.p, bus.p_meas) annotation(
            Line(points = {{10, -80}, {60, -80}, {60, 0}, {100, 0}}, color = {0, 0, 127}));
          annotation(
            Icon(coordinateSystem(grid = {2, 0})));
        end LynxSampledIdealSensors;
        
        model LynxRealSensors
        outer World.SimOptions opt;
        
        extends Rockets.Internal.PartialSensorsPackage;
        RocketControl.Interfaces.AvionicsBus bus annotation(
              Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealSensors.RealGyroscope realGyroscope(noisy = true, biased = true, quantized = true, limited = true, bias(each displayUnit = "rad/s") = {0.00174532925199433, -0.0008726646259971648, -0.0003490658503988659}, bits = 16, fixedLocalSeed = {21, 201, 2001}, rate_max(displayUnit = "rad/s") = 4.363323129985824, samplePeriodMs = opt.samplePeriodMs, sigmaARW(displayUnit = "rad/s") = 0.008726646259971648, sigmaRRW(displayUnit = "rad/s2") = 0.0174532925199433)  annotation(
            Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealSensors.RealAccelerometer realAccelerometer(noisy = true, biased = true, quantized = true, limited = true, acc_max = 156.96, bias = {0.02, 0.04, -0.03}, bits = 16, fixedLocalSeed = {22, 202, 2002}, samplePeriodMs = opt.samplePeriodMs, sigmaBiasInstability = 0.01, sigmaNoise = 0.1)  annotation(
            Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealSensors.RealMagnetometer realMagnetometer(noisy = true, biased = true, quantized = true, limited = true, b_max = 2500000 * 1e-9, bias(displayUnit = "nT") = 250 * 1e-9, bits = 16, fixedLocalSeed = {23, 203, 2003}, misalignement(each displayUnit = "deg") = {from_deg(0.2), from_deg(-0.1), from_deg(0.3)}, samplePeriodMs = opt.samplePeriodMs, sigmaNoise(displayUnit = "nT") = 1.000000000000001e-08) annotation(
            Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealSensors.RealBarometer realBarometer(noisy = true, biased = true, quantized = true, limited = true, bias(displayUnit = "Pa") = 300, bits = 24, fixedLocalSeed = 24, p_max(displayUnit = "Pa") = 110000, p_min(displayUnit = "Pa") = 999.9999999999999, samplePeriodMs = opt.samplePeriodMs, sigmaNoise(displayUnit = "Pa") = 50)  annotation(
            Placement(visible = true, transformation(origin = {0, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealSensors.RealGNSS realGNSS(noisy = true, fixedLocalSeed = {2511, 20511, 200511}, samplePeriodMs = opt.samplePeriodMs, sigmaNoise_vxy = 3, sigmaNoise_vz = 5, sigmaNoise_xy = 15, sigmaNoise_z = 30, sin_error_amplitude = {30, 30, 50}, sin_error_freq = 0.005, sin_error_phase = from_deg({20, 30, 40}))  annotation(
            Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
        connect(frame_a, realGyroscope.frame_a) annotation(
            Line(points = {{-100, 0}, {-40, 0}, {-40, 80}, {-10, 80}}));
        connect(frame_a, realAccelerometer.frame_a) annotation(
            Line(points = {{-100, 0}, {-40, 0}, {-40, 40}, {-10, 40}}));
        connect(frame_a, realMagnetometer.frame_a) annotation(
            Line(points = {{-100, 0}, {-10, 0}}));
        connect(frame_a, realGNSS.frame_a) annotation(
            Line(points = {{-100, 0}, {-40, 0}, {-40, -40}, {-10, -40}}));
        connect(frame_a, realBarometer.frame_a) annotation(
            Line(points = {{-100, 0}, {-40, 0}, {-40, -80}, {-10, -80}}));
        connect(realGyroscope.w, bus.w_meas) annotation(
            Line(points = {{11, 80}, {60, 80}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(realAccelerometer.a, bus.a_meas) annotation(
            Line(points = {{10, 40}, {60, 40}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(realMagnetometer.b, bus.b_meas) annotation(
            Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(realGNSS.x, bus.x_meas) annotation(
            Line(points = {{12, -36}, {60, -36}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(realGNSS.v, bus.v_meas) annotation(
            Line(points = {{12, -44}, {60, -44}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(realBarometer.p, bus.p_meas) annotation(
            Line(points = {{10, -80}, {60, -80}, {60, 0}, {100, 0}}, color = {0, 0, 127}));
          annotation(
            Icon(coordinateSystem(grid = {2, 0})));
        end LynxRealSensors;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Sensors;

      model ContinuousGNC
      extends RocketControl.GNC.Internal.Icons.Navigation;
  Interfaces.AvionicsBus bus annotation(
          Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.LynxIdealSensors lynxIdealSensors annotation(
          Placement(visible = true, transformation(origin = {2, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Navigation.LynxIdealNavigation lynxIdealNavigation annotation(
          Placement(visible = true, transformation(origin = {2, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(frame_a, lynxIdealSensors.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 60}, {-8, 60}}));
  connect(lynxIdealSensors.bus, bus) annotation(
          Line(points = {{12, 60}, {100, 60}, {100, 0}}, thickness = 0.5));
  connect(lynxIdealNavigation.bus, bus) annotation(
          Line(points = {{12, 20}, {100, 20}, {100, 0}}, thickness = 0.5));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end ContinuousGNC;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end GNC;

    package OLD
      model LynxAirframe
        extends RocketControl.Rockets.Internal.PartialAirframe;
        Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.06, I_21 = 0, I_22 = 6.436, I_31 = 0, I_32 = 0, I_33 = 6.437, enforceStates = false, m = 22, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
          Placement(visible = true, transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(r = {-0.02, 0, -0.075}) annotation(
          Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(r = {-0.43, 0, -0.075}) annotation(
          Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.Components.Motors.M2000R m2000r(start_delay = 0.5) annotation(
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
          Icon(graphics = {Polygon(lineColor = {60, 60, 61}, fillColor = {97, 183, 229}, fillPattern = FillPattern.VerticalCylinder, points = {{-20, 60}, {0, 100}, {20, 60}, {20, -60}, {40, -100}, {-40, -100}, {-20, -60}, {-20, 60}}), Text(origin = {-6, 16}, lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name")}));
      end LynxAirframe;

      model SensorsIdeal
        extends Modelica.Icons.RoundSensor;
        extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
        parameter Integer sensorSamplePeriodMs(min = 1) = 20;
        RocketControl.Components.Interfaces.AvionicsBus bus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealGyroscope idealGyroscope(samplePeriodMs = sensorSamplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealAccelerometer idealAccelerometer(samplePeriodMs = sensorSamplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealMagnetometer idealMagnetometer(samplePeriodMs = sensorSamplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealBarometer idealBarometer(samplePeriodMs = sensorSamplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {0, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealGNSS idealGNSS(samplePeriodMs = sensorSamplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealAsset quat_true(samplePeriodMs = sensorSamplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {2, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(frame_a, idealGyroscope.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 90}, {-10, 90}}));
        connect(frame_a, idealAccelerometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 70}, {-10, 70}}));
        connect(frame_a, idealMagnetometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 50}, {-10, 50}}));
        connect(frame_a, idealBarometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 30}, {-10, 30}}));
        connect(frame_a, idealGNSS.frame_a) annotation(
          Line(points = {{-100, 0}, {-10, 0}}));
        connect(idealGyroscope.w, bus.w_meas) annotation(
          Line(points = {{10, 90}, {100, 90}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealAccelerometer.acc, bus.a_meas) annotation(
          Line(points = {{10, 70}, {100, 70}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealMagnetometer.b, bus.b_meas) annotation(
          Line(points = {{10, 50}, {100, 50}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealBarometer.press, bus.p_meas) annotation(
          Line(points = {{10, 24}, {100, 24}, {100, 100}}, color = {0, 0, 127}));
        connect(idealGNSS.pos, bus.x_meas) annotation(
          Line(points = {{12, 4}, {100, 4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGNSS.vel, bus.v_meas) annotation(
          Line(points = {{12, -4}, {100, -4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGyroscope.w, bus.w_est) annotation(
          Line(points = {{10, 90}, {100, 90}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGNSS.pos, bus.x_est) annotation(
          Line(points = {{12, 4}, {100, 4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGNSS.vel, bus.v_est) annotation(
          Line(points = {{12, -4}, {100, -4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(frame_a, quat_true.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, -30}, {-8, -30}}));
        connect(quat_true.q, bus.q_est) annotation(
          Line(points = {{14, -30}, {100, -30}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      protected
        annotation(
          Icon(graphics = {Line(origin = {-83, 0}, points = {{-13, 0}, {13, 0}}), Text(origin = {-1.42109e-14, -40}, extent = {{-60, 20}, {60, -20}}, textString = "sens"), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name"), Line(origin = {85, 0}, points = {{-15, 0}, {9, 0}, {15, 0}})}));
      end SensorsIdeal;

      model Sensors
        extends Modelica.Icons.RoundSensor;
        extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
        parameter Integer sensorSamplePeriodMs(min = 1) = 20;
        RocketControl.Components.Sensors.RealMagnetometer realMagnetometer(b_max = 2500000 * 1e-9, bias(displayUnit = "nT") = 250 * 1e-9, bits = 16, fixedLocalSeed = {99, 432, 543543}, misalignement(each displayUnit = "deg") = {from_deg(0.2), from_deg(-0.1), from_deg(0.3)}, samplePeriodMs = sensorSamplePeriodMs, sigmaNoise(displayUnit = "nT") = 1.000000000000001e-08) annotation(
          Placement(visible = true, transformation(origin = {-10, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealGyroscope realGyroscope(bias(each displayUnit = "rad/s") = {0.00174532925199433, -0.0008726646259971648, -0.0003490658503988659}, bits = 16, fixedLocalSeed = {9, 423, 43214321}, rate_max(displayUnit = "rad/s") = 4.363323129985824, samplePeriodMs = sensorSamplePeriodMs, sigmaARW(displayUnit = "rad/s") = 0.008726646259971648, sigmaRRW(displayUnit = "rad/s2") = 0.0174532925199433) annotation(
          Placement(visible = true, transformation(origin = {-10, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealAccelerometer realAccelerometer(acc_max = 156.96, bias = {0.02, 0.04, -0.03}, bits = 16, fixedLocalSeed = {637914168, 1062993719, 2034216499}, samplePeriodMs = sensorSamplePeriodMs, sigmaBiasInstability = 0.01, sigmaNoise = 0.1) annotation(
          Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealBarometer realBarometer(bias(displayUnit = "Pa") = 300, bits = 24, fixedLocalSeed = 5325587, p_max(displayUnit = "Pa") = 110000, p_min(displayUnit = "Pa") = 999.9999999999999, samplePeriodMs = sensorSamplePeriodMs, sigmaNoise(displayUnit = "Pa") = 50) annotation(
          Placement(visible = true, transformation(origin = {-10, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.RealGNSS realGNSS(fixedLocalSeed = {1775435783, 568478634, -1728550798}, samplePeriodMs = sensorSamplePeriodMs, sigmaNoise_vxy = 3, sigmaNoise_vz = 5, sigmaNoise_xy = 15, sigmaNoise_z = 30, sin_error_amplitude = {30, 30, 50}, sin_error_freq = 0.005, sin_error_phase = from_deg({20, 30, 40})) annotation(
          Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Interfaces.AvionicsBus bus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(frame_a, realMagnetometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 46}, {-20, 46}}));
        connect(frame_a, realGyroscope.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 68}, {-20, 68}}));
        connect(frame_a, realAccelerometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 90}, {-20, 90}}));
        connect(realBarometer.frame_a, frame_a) annotation(
          Line(points = {{-20, 24}, {-40, 24}, {-40, 0}, {-100, 0}}));
        connect(realGNSS.frame_a, frame_a) annotation(
          Line(points = {{-20, 0}, {-100, 0}}, color = {95, 95, 95}));
        connect(realAccelerometer.acc, bus.a_meas) annotation(
          Line(points = {{0, 90}, {100, 90}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(realGyroscope.w_meas, bus.w_meas) annotation(
          Line(points = {{0, 68}, {100, 68}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(realMagnetometer.b_meas, bus.b_meas) annotation(
          Line(points = {{0, 46}, {100, 46}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(realBarometer.press, bus.p_meas) annotation(
          Line(points = {{0, 24}, {100, 24}, {100, 100}}, color = {0, 0, 127}));
        connect(realGNSS.pos, bus.x_meas) annotation(
          Line(points = {{1, 4}, {100, 4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(realGNSS.vel, bus.v_meas) annotation(
          Line(points = {{1, -4}, {100, -4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
      protected
        annotation(
          Icon(graphics = {Line(origin = {-83, 0}, points = {{-13, 0}, {13, 0}}), Text(origin = {-1.42109e-14, -40}, extent = {{-60, 20}, {60, -20}}, textString = "sens"), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name"), Line(origin = {85, 0}, points = {{-15, 0}, {9, 0}, {15, 0}})}));
      end Sensors;

      model Estimators
        extends RocketControl.GNC.Internal.Icons.Navigation;
        parameter Integer samplePeriodMs(min = 1) = 20;
        RocketControl.Components.Interfaces.AvionicsBus bus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Navigation.AttitudeEstimation attitudeEstimation(elevation0 = 1.466076571675237, heading0 = 2.268928027592628, samplingPeriodMs = samplePeriodMs, sigma_b = 2, sigma_u = from_deg(10), sigma_v = from_deg(60)) annotation(
          Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Navigation.PositionEstimation positionEstimation(samplingPeriodMs = samplePeriodMs, sigma_gps = {1000, 1000, 1000, 100, 100, 100}, sigma_pos = 2, sigma_vel = 1) annotation(
          Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Quaternion.Quaternion2Euler eul_est annotation(
          Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.UnwrapAngle eul_est_unwrap(n = 3) annotation(
          Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(bus.w_meas, attitudeEstimation.w_meas) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 58}, {-62, 58}}, thickness = 0.5));
        connect(bus.b_meas, attitudeEstimation.b_meas) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 50}, {-62, 50}}, thickness = 0.5));
        connect(bus.q_est, positionEstimation.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 20}, {-62, 20}}, thickness = 0.5));
        connect(bus.a_meas, positionEstimation.acc_body) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 14}, {-62, 14}}, thickness = 0.5));
        connect(bus.x_meas, positionEstimation.pos_ned) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 6}, {-62, 6}}, thickness = 0.5));
        connect(bus.v_meas, positionEstimation.vel_ned) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 0}, {-62, 0}}, thickness = 0.5));
        connect(bus.x_est, attitudeEstimation.r_0_est) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 42}, {-62, 42}}, thickness = 0.5));
        connect(attitudeEstimation.q_est, bus.q_est) annotation(
          Line(points = {{-38, 56}, {100, 56}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(attitudeEstimation.w_est, bus.w_est) annotation(
          Line(points = {{-38, 46}, {100, 46}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(positionEstimation.pos_est, bus.x_est) annotation(
          Line(points = {{-38, 14}, {100, 14}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(positionEstimation.vel_est, bus.v_est) annotation(
          Line(points = {{-38, 6}, {100, 6}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(bus.q_est, eul_est.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -30}, {-62, -30}}, thickness = 0.5));
        connect(eul_est.eul, eul_est_unwrap.angle) annotation(
          Line(points = {{-38, -30}, {-22, -30}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Line(origin = {92, 0}, points = {{-2, 0}, {2, 0}})}),
          Diagram);
      end Estimators;

      model Controllers
        RocketControl.GNC.Control.Autopilot autopilot(CLLdr = 0.0388, CLMalpha = -0.6798, CLMdp = 0.3573, CLNbeta = 0.6798, CLNdy = 0.3573, CNalpha = 0.4158, CNdp = 0.0557, CYbeta = -0.4158, CYdy = 0.0557, S = 0.0177, c = 0.15, fin_max_angle = 10) annotation(
          Placement(visible = true, transformation(origin = {78, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain P(k = 20 * 180 / pi) annotation(
          Placement(visible = true, transformation(origin = {2, 50}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        Components.Blocks.dynamicPressure dynamicPressure annotation(
          Placement(visible = true, transformation(origin = {18, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Blocks.Track track annotation(
          Placement(visible = true, transformation(origin = {-70, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.WrapAngle wrapAngle annotation(
          Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
        Modelica.Blocks.Math.UnitConversions.From_deg from_deg annotation(
          Placement(visible = true, transformation(origin = {-68, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body f_body annotation(
          Placement(visible = true, transformation(origin = {50, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback feedback1(y(quantity = "Angle", unit = "rad", displayUnit = "deg"), u1(quantity = "Angle", unit = "rad", displayUnit = "deg"), u2(quantity = "Angle", unit = "rad", displayUnit = "deg")) annotation(
          Placement(visible = true, transformation(origin = {-40, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
          Placement(visible = true, transformation(origin = {-68, 90}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = 0.1 * 180 / pi) annotation(
          Placement(visible = true, transformation(origin = {16, 90}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Math.Blocks.VectorConstant vectorConstant(k = {0, 0, 1}) annotation(
          Placement(visible = true, transformation(origin = {-69, 25}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Math.Blocks.UnitVector vu annotation(
          Placement(visible = true, transformation(origin = {-69, 5}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Math.Blocks.CrossProduct crossProduct annotation(
          Placement(visible = true, transformation(origin = {-40, 12}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.Math.Blocks.VectorGain vectorGain annotation(
          Placement(visible = true, transformation(origin = {22, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
        RocketControl.Math.Blocks.ProjectPlane projectPlane(n = {0, 0, 1}) annotation(
          Placement(visible = true, transformation(origin = {-87, 5}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Math.Blocks.SmallestAngleDiff smallestAngleDiff annotation(
          Placement(visible = true, transformation(origin = {-19, 51}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant constant2(k = 270) annotation(
          Placement(visible = true, transformation(origin = {-90, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      equation
        connect(dynamicPressure.q, autopilot.q) annotation(
          Line(points = {{30, -76}, {66, -76}, {66, -8}}, color = {0, 0, 127}));
        connect(avionicsBus.x_meas, dynamicPressure.pos) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -70}, {6, -70}}));
        connect(from_deg.y, wrapAngle.u) annotation(
          Line(points = {{-64, 50}, {-54, 50}}, color = {0, 0, 127}));
        connect(f_body.x_b[3], autopilot.Fn) annotation(
          Line(points = {{61, 44}, {61, 30}, {66, 30}, {66, 8}}, color = {0, 0, 127}));
        connect(f_body.x_b[2], autopilot.Fy) annotation(
          Line(points = {{61, 44}, {61, 28}, {66, 28}, {66, 4}}, color = {0, 0, 127}));
        connect(constant1.y, feedback1.u1) annotation(
          Line(points = {{-64, 90}, {-48, 90}}, color = {0, 0, 127}));
        connect(feedback1.y, gain.u) annotation(
          Line(points = {{-30, 90}, {9, 90}}, color = {0, 0, 127}));
        connect(gain.y, autopilot.Ml) annotation(
          Line(points = {{23, 90}, {66, 90}, {66, 0}}, color = {0, 0, 127}));
        connect(track.v, avionicsBus.v_est) annotation(
          Line(points = {{-82, -40}, {-100, -40}, {-100, 100}, {100, 100}}));
        connect(avionicsBus.q_est, f_body.q_bw) annotation(
          Line(points = {{100, 100}, {38, 100}, {38, 38}}));
        connect(avionicsBus.v_est, dynamicPressure.vel) annotation(
          Line(points = {{100, 100}, {38, 100}, {38, 46}, {46, 46}}));
        connect(avionicsBus.w_est[1], feedback1.u2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 68}, {-40, 68}, {-40, 82}}));
        connect(autopilot.deflection, fin) annotation(
          Line(points = {{89, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorGain.vk, f_body.x_w) annotation(
          Line(points = {{26, 50}, {38, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(P.y, vectorGain.k) annotation(
          Line(points = {{8, 50}, {11.5, 50}, {11.5, 48}, {17, 48}}, color = {0, 0, 127}));
        connect(crossProduct.vc, vectorGain.v) annotation(
          Line(points = {{-34, 12}, {17, 12}, {17, 52}}, color = {0, 0, 127}));
        connect(projectPlane.vp, vu.v) annotation(
          Line(points = {{-81.5, 5}, {-75, 5}}, color = {0, 0, 127}));
        connect(projectPlane.v, track.v) annotation(
          Line(points = {{-93, 5}, {-100, 5}, {-100, -40}, {-82, -40}}, color = {0, 0, 127}));
        connect(vu.vu, crossProduct.v2) annotation(
          Line(points = {{-64, 6}, {-48, 6}, {-48, 10}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConstant.v, crossProduct.v1) annotation(
          Line(points = {{-64, 26}, {-50, 26}, {-50, 14}, {-48, 14}}, color = {0, 0, 127}, thickness = 0.5));
        connect(wrapAngle.y, smallestAngleDiff.a1) annotation(
          Line(points = {{-46, 50}, {-38, 50}, {-38, 55}, {-27, 55}}, color = {0, 0, 127}));
        connect(smallestAngleDiff.amin, P.u) annotation(
          Line(points = {{-11, 51}, {-12, 51}, {-12, 50}, {-6, 50}}, color = {0, 0, 127}));
        connect(smallestAngleDiff.a2, track.track) annotation(
          Line(points = {{-27, 47}, {-38, 47}, {-38, 26}, {-22, 26}, {-22, -40}, {-58, -40}}, color = {0, 0, 127}));
        connect(constant2.y, from_deg.u) annotation(
          Line(points = {{-86, 50}, {-73, 50}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}, endAngle = 360), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end Controllers;

      model ControllersGlide
        RocketControl.GNC.Control.Autopilot autopilot(CLLdr = 0.0388, CLMalpha = -0.6798, CLMdp = 0.3573, CLNbeta = 0.6798, CLNdy = 0.3573, CNalpha = 0.4158, CNdp = 0.0557, CYbeta = -0.4158, CYdy = 0.0557, S = 0.0177, c = 0.15, fin_max_angle = 10) annotation(
          Placement(visible = true, transformation(origin = {78, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.Blocks.dynamicPressure dynamicPressure annotation(
          Placement(visible = true, transformation(origin = {18, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Rockets.Lynx.Control.GlideControl glideControl annotation(
          Placement(visible = true, transformation(origin = {2, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
          Placement(visible = true, transformation(origin = {30, -44}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold = 29) annotation(
          Placement(visible = true, transformation(origin = {-55, -15}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        Modelica.Blocks.Sources.ContinuousClock continuousClock annotation(
          Placement(visible = true, transformation(origin = {-89, -15}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Rockets.Lynx.Control.TrackControl trackControl annotation(
          Placement(visible = true, transformation(origin = {10, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Add add annotation(
          Placement(visible = true, transformation(origin = {36, 30}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
        Modelica.Blocks.Math.Add add1 annotation(
          Placement(visible = true, transformation(origin = {36, 12}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
        RocketControl.Rockets.Lynx.Control.LandTarget landTarget(coast_glide_angle = 20 * pi / 180, target = {-2944, 4436, 0}) annotation(
          Placement(visible = true, transformation(origin = {-66, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(dynamicPressure.q, autopilot.q) annotation(
          Line(points = {{30, -76}, {66, -76}, {66, -8}}, color = {0, 0, 127}));
        connect(avionicsBus.x_meas, dynamicPressure.pos) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -70}, {6, -70}}));
        connect(autopilot.deflection, fin) annotation(
          Line(points = {{89, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus, glideControl.avionicsBus) annotation(
          Line(points = {{100, 100}, {-8, 100}, {-8, 29}}));
        connect(constant1.y, autopilot.Ml) annotation(
          Line(points = {{34, -44}, {40, -44}, {40, 0}, {66, 0}}, color = {0, 0, 127}));
        connect(avionicsBus.v_est, dynamicPressure.vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -80}, {6, -80}}));
        connect(greaterThreshold.y, glideControl.u) annotation(
          Line(points = {{-49.5, -15}, {-10, -15}, {-10, 12}}, color = {255, 0, 255}));
        connect(continuousClock.y, greaterThreshold.u) annotation(
          Line(points = {{-83.5, -15}, {-70.5, -15}, {-70.5, -14}, {-60, -14}}, color = {0, 0, 127}));
        connect(avionicsBus, trackControl.avionicsBus) annotation(
          Line(points = {{100, 100}, {0, 100}, {0, 65}}));
        connect(add.y, autopilot.Fn) annotation(
          Line(points = {{40, 30}, {66, 30}, {66, 8}}, color = {0, 0, 127}));
        connect(add1.y, autopilot.Fy) annotation(
          Line(points = {{40, 12}, {58, 12}, {58, 4}, {66, 4}}, color = {0, 0, 127}));
        connect(trackControl.f_b[3], add.u1) annotation(
          Line(points = {{21, 56}, {32, 56}, {32, 32}}, color = {0, 0, 127}));
        connect(trackControl.f_b[2], add1.u1) annotation(
          Line(points = {{21, 56}, {26, 56}, {26, 14}, {32, 14}}, color = {0, 0, 127}));
        connect(glideControl.f_b[3], add.u2) annotation(
          Line(points = {{13, 20}, {32, 20}, {32, 28}}, color = {0, 0, 127}));
        connect(glideControl.f_b[2], add1.u2) annotation(
          Line(points = {{13, 20}, {26, 20}, {26, 10}, {32, 10}}, color = {0, 0, 127}));
        connect(landTarget.track, trackControl.track_ref) annotation(
          Line(points = {{-54, 50}, {-2, 50}, {-2, 56}}, color = {0, 0, 127}));
        connect(landTarget.glideangle, glideControl.angle_ref) annotation(
          Line(points = {{-54, 40}, {-30, 40}, {-30, 20}, {-10, 20}}, color = {0, 0, 127}));
        connect(avionicsBus, landTarget.bus) annotation(
          Line(points = {{100, 100}, {-76, 100}, {-76, 44}}));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}, endAngle = 360), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersGlide;

      model ControllersTrack
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Blocks.dynamicPressure dynamicPressure annotation(
          Placement(visible = true, transformation(origin = {-50, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.x_est, dynamicPressure.pos) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -66}, {-62, -66}}, thickness = 0.5));
        connect(avionicsBus.v_est, dynamicPressure.vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -76}, {-62, -76}}, thickness = 0.5));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersTrack;

      model GlideControl
      equation

        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end GlideControl;

      package Control
        model TrackControl
          Modelica.Blocks.Interfaces.RealOutput f_b[3] annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput track_ref annotation(
            Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
            Placement(visible = true, transformation(origin = {-94, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.CrossProduct crossProduct annotation(
            Placement(visible = true, transformation(origin = {-22, -58}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
          RocketControl.Math.Blocks.SmallestAngleDiff smallestAngleDiff annotation(
            Placement(visible = true, transformation(origin = {-18, 30}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.ProjectOnPlane projectPlane(n = {0, 0, 1}) annotation(
            Placement(visible = true, transformation(origin = {-69, -65}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
          Modelica.Blocks.Math.UnitConversions.From_deg from_deg annotation(
            Placement(visible = true, transformation(origin = {-68, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.VectorGain vectorGain annotation(
            Placement(visible = true, transformation(origin = {40, 6}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
          RocketControl.Components.Blocks.ned2body f_body annotation(
            Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 1}) annotation(
            Placement(visible = true, transformation(origin = {-51, -45}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
          RocketControl.Components.Blocks.Track track annotation(
            Placement(visible = true, transformation(origin = {-66, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Math.Gain P(k = 10 * 180 / pi) annotation(
            Placement(visible = true, transformation(origin = {16, 30}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
          Modelica.Blocks.Math.WrapAngle wrapAngle annotation(
            Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.UnitVector vu annotation(
            Placement(visible = true, transformation(origin = {-51, -65}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        equation
          connect(crossProduct.vc, vectorGain.v) annotation(
            Line(points = {{-15, -58}, {-7.5, -58}, {-7.5, 8}, {35, 8}}, color = {0, 0, 127}));
          connect(vu.vu, crossProduct.v2) annotation(
            Line(points = {{-45.5, -65}, {-29.5, -65}, {-29.5, -61}}, color = {0, 0, 127}, thickness = 0.5));
          connect(projectPlane.vp, vu.v) annotation(
            Line(points = {{-63.5, -65}, {-57, -65}}, color = {0, 0, 127}));
          connect(smallestAngleDiff.amin, P.u) annotation(
            Line(points = {{-9, 30}, {9, 30}}, color = {0, 0, 127}));
          connect(wrapAngle.y, smallestAngleDiff.a1) annotation(
            Line(points = {{-46, 50}, {-38, 50}, {-38, 35}, {-28, 35}}, color = {0, 0, 127}));
          connect(vectorGain.vk, f_body.x_w) annotation(
            Line(points = {{44, 6}, {58, 6}}, color = {0, 0, 127}, thickness = 0.5));
          connect(projectPlane.v, track.v) annotation(
            Line(points = {{-75, -65}, {-78, -65}, {-78, 24}}, color = {0, 0, 127}));
          connect(from_deg.y, wrapAngle.u) annotation(
            Line(points = {{-64, 50}, {-54, 50}}, color = {0, 0, 127}));
          connect(vectorConstant.v, crossProduct.v1) annotation(
            Line(points = {{-45.5, -45}, {-31.5, -45}, {-31.5, -57}, {-29.5, -57}}, color = {0, 0, 127}, thickness = 0.5));
          connect(f_body.x_b, f_b) annotation(
            Line(points = {{82, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
          connect(track.track, smallestAngleDiff.a2) annotation(
            Line(points = {{-55, 24}, {-41.5, 24}, {-41.5, 26}, {-28, 26}}, color = {0, 0, 127}));
          connect(from_deg.u, track_ref) annotation(
            Line(points = {{-72, 50}, {-96, 50}, {-96, 40}, {-120, 40}}, color = {0, 0, 127}));
          connect(avionicsBus.v_est, track.v) annotation(
            Line(points = {{0, 0}, {-92, 0}, {-92, 24}, {-78, 24}}));
          connect(P.y, vectorGain.k) annotation(
            Line(points = {{22, 30}, {30, 30}, {30, 4}, {36, 4}}, color = {0, 0, 127}));
          connect(avionicsBus.q_est, f_body.q_bw) annotation(
            Line(points = {{0, 0}, {-92, 0}, {-92, -6}, {58, -6}}));
          annotation(
            Icon(graphics = {Text(origin = {0, 1}, extent = {{-100, 97}, {100, -97}}, textString = "track")}));
        end TrackControl;

        model GlideControl
          RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
            Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput f_b[3] annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          RocketControl.Components.Blocks.ned2body f_body annotation(
            Placement(visible = true, transformation(origin = {62, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput angle_ref annotation(
            Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.VectorGain vectorGain annotation(
            Placement(visible = true, transformation(origin = {36, 10}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
          RocketControl.Math.Blocks.SmallestAngleDiff smallestAngleDiff annotation(
            Placement(visible = true, transformation(origin = {-28, 34}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.VectorConstant z(k = {0, 0, 1}) annotation(
            Placement(visible = true, transformation(origin = {-77, -29}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.CrossProduct crossProduct annotation(
            Placement(visible = true, transformation(origin = {-48, -36}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
          RocketControl.Components.Blocks.GlideAngle glideAngle annotation(
            Placement(visible = true, transformation(origin = {-57, -1}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.CrossProduct crossProduct1 annotation(
            Placement(visible = true, transformation(origin = {-18, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
          RocketControl.Math.Blocks.Vector.UnitVector unitVector annotation(
            Placement(visible = true, transformation(origin = {4, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
          Modelica.Blocks.Math.UnitConversions.From_deg from_deg annotation(
            Placement(visible = true, transformation(origin = {-80, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
          Modelica.Blocks.Math.WrapAngle wrapAngle annotation(
            Placement(visible = true, transformation(origin = {-62, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
          Modelica.Blocks.Interfaces.BooleanInput u annotation(
            Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Logical.Switch switch1 annotation(
            Placement(visible = true, transformation(origin = {-3, 31}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
          Modelica.Blocks.Sources.Constant constant2(k = 0) annotation(
            Placement(visible = true, transformation(origin = {-32, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
          Modelica.Blocks.Continuous.PI pi(T = 3, k = 25 * 180 / 3.14) annotation(
            Placement(visible = true, transformation(origin = {22, 32}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        equation
          connect(z.v, crossProduct.v1) annotation(
            Line(points = {{-71.5, -29}, {-63.5, -29}, {-63.5, -34}, {-55, -34}}, color = {0, 0, 127}, thickness = 0.5));
          connect(glideAngle.angle, smallestAngleDiff.a2) annotation(
            Line(points = {{-50, 0}, {-38, 0}, {-38, 30}}, color = {0, 0, 127}));
          connect(vectorGain.vk, f_body.x_w) annotation(
            Line(points = {{40, 10}, {50, 10}, {50, 6}}, color = {0, 0, 127}, thickness = 0.5));
          connect(f_body.x_b, f_b) annotation(
            Line(points = {{73, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
          connect(crossProduct1.vc, unitVector.v) annotation(
            Line(points = {{-12, -40}, {-3, -40}}, color = {0, 0, 127}, thickness = 0.5));
          connect(unitVector.vu, vectorGain.v) annotation(
            Line(points = {{10, -40}, {20, -40}, {20, 12}, {31, 12}}, color = {0, 0, 127}, thickness = 0.5));
          connect(crossProduct.vc, crossProduct1.v1) annotation(
            Line(points = {{-42, -36}, {-26, -36}, {-26, -38}}, color = {0, 0, 127}, thickness = 0.5));
          connect(from_deg.y, wrapAngle.u) annotation(
            Line(points = {{-75.6, 40}, {-65.6, 40}}, color = {0, 0, 127}));
          connect(angle_ref, from_deg.u) annotation(
            Line(points = {{-120, 40}, {-84, 40}}, color = {0, 0, 127}));
          connect(wrapAngle.y, smallestAngleDiff.a1) annotation(
            Line(points = {{-58, 40}, {-48, 40}, {-48, 38}, {-38, 38}}, color = {0, 0, 127}));
          connect(smallestAngleDiff.amin, switch1.u1) annotation(
            Line(points = {{-20, 34}, {-8, 34}, {-8, 36}}, color = {0, 0, 127}));
          connect(constant2.y, switch1.u3) annotation(
            Line(points = {{-28, -10}, {-8, -10}, {-8, 28}}, color = {0, 0, 127}));
          connect(switch1.u2, u) annotation(
            Line(points = {{-8, 32}, {-22, 32}, {-22, -80}, {-120, -80}}, color = {255, 0, 255}));
          connect(switch1.y, pi.u) annotation(
            Line(points = {{2, 32}, {15, 32}}, color = {0, 0, 127}));
          connect(pi.y, vectorGain.k) annotation(
            Line(points = {{29, 32}, {32, 32}, {32, 8}}, color = {0, 0, 127}));
          connect(avionicsBus.v_est, glideAngle.v) annotation(
            Line(points = {{-110, 0}, {-66, 0}}, thickness = 0.5));
          connect(avionicsBus.v_est, crossProduct.v2) annotation(
            Line(points = {{-110, 0}, {-92, 0}, {-92, -46}, {-66, -46}, {-66, -38}, {-56, -38}}, thickness = 0.5));
          connect(avionicsBus.v_est, crossProduct1.v2) annotation(
            Line(points = {{-110, 0}, {-92, 0}, {-92, -46}, {-32, -46}, {-32, -42}, {-26, -42}}, thickness = 0.5));
          connect(avionicsBus.q_est, f_body.q_bw) annotation(
            Line(points = {{-110, 0}, {-92, 0}, {-92, -60}, {40, -60}, {40, -6}, {50, -6}}, thickness = 0.5));
          annotation(
            Icon(coordinateSystem(grid = {2, 0})));
        end GlideControl;

        model LandTarget
          parameter SI.Position[3] target;
          parameter SI.Angle coast_glide_angle(displayUnit = "deg");
          parameter SI.Position x_small = 1e-5;
          Modelica.Blocks.Interfaces.RealOutput track annotation(
            Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput glideangle annotation(
            Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          RocketControl.Components.Interfaces.AvionicsBus bus annotation(
            Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          SI.Position[3] x_r;
          SI.Position x_r_norm;
        equation
          x_r = target - bus.x_meas;
          x_r_norm = norm(x_r);
  if noEvent(x_r_norm > x_small) then
            glideangle = to_deg(-asin(x_r[3] / x_r_norm));
//    glideangle = -15;
          else
            glideangle = 0;
          end if;
//  track = 0;
          if noEvent(abs(x_r[1]) > x_small) then
            track = to_deg(atan2(x_r[2], x_r[1]));
          else
            track = to_deg(sign(x_r[2]) * pi / 2);
          end if;
          annotation(
            Icon(coordinateSystem(grid = {2, 0})));
        end LandTarget;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Control;

      model Errors
        parameter Integer sensorSamplePeriodMs(min = 1) = 20;
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealAsset quat_true(samplePeriodMs = sensorSamplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.IdealGNSS gnss_true(samplePeriodMs = sensorSamplePeriodMs) annotation(
          Placement(visible = true, transformation(origin = {-40, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorAdd pos_err(gain = {1, -1}) annotation(
          Placement(visible = true, transformation(origin = {22, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorAdd vel_err(gain = {1, -1}) annotation(
          Placement(visible = true, transformation(origin = {22, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Quaternion.QuaternionError quat_err annotation(
          Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Quaternion.Quaternion2Euler eul_err annotation(
          Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.UnwrapAngle eul_err_unwrap(n = 3) annotation(
          Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Quaternion.Quaternion2Euler eul_true annotation(
          Placement(visible = true, transformation(origin = {10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.UnwrapAngle eul_true_unwrap(n = 3) annotation(
          Placement(visible = true, transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorAdd target_err(gain = {1, -1}) annotation(
          Placement(visible = true, transformation(origin = {10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {-2944, 4436, 0}) annotation(
          Placement(visible = true, transformation(origin = {-50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Math.Blocks.Vector.VectorNorm target_distance annotation(
          Placement(visible = true, transformation(origin = {56, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(frame_a, gnss_true.frame_a) annotation(
          Line(points = {{-100, 0}, {-70, 0}, {-70, 60}, {-50, 60}}));
        connect(frame_a, quat_true.frame_a) annotation(
          Line(points = {{-100, 0}, {-50, 0}}));
        connect(gnss_true.pos, pos_err.v1) annotation(
          Line(points = {{-28, 64}, {-20, 64}, {-20, 94}, {10, 94}}, color = {0, 0, 127}, thickness = 0.5));
        connect(gnss_true.vel, vel_err.v1) annotation(
          Line(points = {{-28, 56}, {-9, 56}, {-9, 54}, {10, 54}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.x_est, pos_err.v2) annotation(
          Line(points = {{-100, 60}, {-100, 100}, {-10, 100}, {-10, 86}, {10, 86}}, thickness = 0.5));
        connect(avionicsBus.v_est, vel_err.v2) annotation(
          Line(points = {{-100, 60}, {-100, 100}, {-10, 100}, {-10, 46}, {10, 46}}, thickness = 0.5));
        connect(quat_true.q, quat_err.q1) annotation(
          Line(points = {{-28, 0}, {-20, 0}, {-20, 4}, {-2, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(quat_err.qerr, eul_err.q) annotation(
          Line(points = {{21, 0}, {38, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(eul_err.eul, eul_err_unwrap.angle) annotation(
          Line(points = {{62, 0}, {78, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, quat_err.q2) annotation(
          Line(points = {{-100, 60}, {-100, 100}, {-10, 100}, {-10, -4}, {-2, -4}}, thickness = 0.5));
        connect(eul_true.q, quat_true.q) annotation(
          Line(points = {{-2, -30}, {-20, -30}, {-20, 0}, {-28, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(eul_true.eul, eul_true_unwrap.angle) annotation(
          Line(points = {{22, -30}, {38, -30}}, color = {0, 0, 127}, thickness = 0.5));
        connect(gnss_true.pos, target_err.v1) annotation(
          Line(points = {{-28, 64}, {-20, 64}, {-20, -66}, {-2, -66}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConstant.v, target_err.v2) annotation(
          Line(points = {{-38, -70}, {-30, -70}, {-30, -74}, {-2, -74}}, color = {0, 0, 127}, thickness = 0.5));
        connect(target_err.vc, target_distance.v) annotation(
          Line(points = {{22, -70}, {44, -70}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(fillColor = {199, 227, 235}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {0, 13}, extent = {{-100, 87}, {100, -87}}, textString = "e"), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
      end Errors;

      model ControllersLQ
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.RocketAndActuator systemMatrices(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22, wa = 13) annotation(
          Placement(visible = true, transformation(origin = {-50, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 9, val = diagonal({0, 10, 10, 100, 0, 0, 0, 0, 0} * 100)) annotation(
          Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({1, 1, 0.4} * 20)) annotation(
          Placement(visible = true, transformation(origin = {-50, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, n = 9, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {8, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate(n1 = 3) annotation(
          Placement(visible = true, transformation(origin = {-70, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body annotation(
          Placement(visible = true, transformation(origin = {-16, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {44, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {86, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Rockets.Lynx.AccGuidance accGuidance(useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {-10, -98}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 6) annotation(
          Placement(visible = true, transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant1(k = {0, 0, 0}, n = 3) annotation(
          Placement(visible = true, transformation(origin = {-180, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, vectorConcatenate.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -60}, {-82, -60}}, thickness = 0.5));
        connect(systemMatrices.A, continuousLQR.A) annotation(
          Line(points = {{-39, 61}, {-10, 61}, {-10, 8}, {-4, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.B, continuousLQR.B) annotation(
          Line(points = {{-39, 51}, {-14, 51}, {-14, 4}, {-4, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{-38, 10}, {-18, 10}, {-18, 0}, {-4, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{-39, -16}, {-14, -16}, {-14, -4}, {-4, -4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, ned2body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 86}, {-4, 86}}, thickness = 0.5));
        connect(avionicsBus.v_est, ned2body.x_w) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 74}, {-4, 74}}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQR.enable) annotation(
          Line(points = {{100, 100}, {8, 100}, {8, 10}}, color = {255, 0, 255}));
        connect(continuousLQR.u, control2Deflection.u) annotation(
          Line(points = {{20, 0}, {32, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ned2body.x_b[1], vectorConcatenate.v1[1]) annotation(
          Line(points = {{-26, 80}, {-82, 80}, {-82, -52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus, accGuidance.bus) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -98}, {-20, -98}}, thickness = 0.5));
        connect(accGuidance.acc_err_int[1], vectorConcatenate.v1[2]) annotation(
          Line(points = {{1, -98}, {10, -98}, {10, -46}, {-82, -46}, {-82, -52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(accGuidance.acc_err_int[2], vectorConcatenate.v1[3]) annotation(
          Line(points = {{1, -98}, {10, -98}, {10, -46}, {-82, -46}, {-82, -52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, accGuidance.enable) annotation(
          Line(points = {{100, 100}, {-10, 100}, {-10, -89}}, color = {255, 0, 255}));
        connect(continuousLQR.u, avionicsBus.control_cmd) annotation(
          Line(points = {{20, 0}, {26, 0}, {26, 100}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-58, -56}, {-52, -56}, {-52, -66}, {-42, -66}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{-18, -70}, {-12, -70}, {-12, -8}, {-4, -8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.control_cmd, vectorConcatenate1.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -74}, {-42, -74}}, thickness = 0.5));
        connect(ned2body.x_b, systemMatrices.vel) annotation(
          Line(points = {{-26, 80}, {-82, 80}, {-82, 66}, {-62, 66}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatrices.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 60}, {-62, 60}}, thickness = 0.5));
        connect(avionicsBus.x_est, systemMatrices.x_est) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 54}, {-62, 54}}, thickness = 0.5));
        connect(vectorConstant1.v, systemMatrices.control_cmd) annotation(
          Line(points = {{-168, 64}, {-134, 64}, {-134, 48}, {-62, 48}}, color = {0, 0, 127}, thickness = 0.5));
        connect(control2Deflection.deflection, fin) annotation(
          Line(points = {{56, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersLQ;

      model SensorsIdealCont
        extends Modelica.Icons.RoundSensor;
        extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
        RocketControl.Components.Interfaces.AvionicsBus bus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.Continuous.IdealGyroscope idealGyroscope annotation(
          Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.Continuous.IdealAccelerometer idealAccelerometer annotation(
          Placement(visible = true, transformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.Continuous.IdealMagnetometer idealMagnetometer annotation(
          Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.Continuous.IdealBarometer idealBarometer annotation(
          Placement(visible = true, transformation(origin = {0, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.Continuous.IdealGNSS idealGNSS annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.Continuous.IdealAsset quat_true annotation(
          Placement(visible = true, transformation(origin = {2, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Math.Blocks.Quaternion2Euler quaternion2Euler annotation(
          Placement(visible = true, transformation(origin = {66, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Blocks.EulerRates eulerRates annotation(
          Placement(visible = true, transformation(origin = {24, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(frame_a, idealGyroscope.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 90}, {-10, 90}}));
        connect(frame_a, idealAccelerometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 70}, {-10, 70}}));
        connect(frame_a, idealMagnetometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 50}, {-10, 50}}));
        connect(frame_a, idealBarometer.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, 30}, {-10, 30}}));
        connect(frame_a, idealGNSS.frame_a) annotation(
          Line(points = {{-100, 0}, {-10, 0}}));
        connect(idealGyroscope.w, bus.w_meas) annotation(
          Line(points = {{10, 90}, {100, 90}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealAccelerometer.acc, bus.a_meas) annotation(
          Line(points = {{10, 70}, {100, 70}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealMagnetometer.b, bus.b_meas) annotation(
          Line(points = {{10, 50}, {100, 50}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealBarometer.press, bus.p_meas) annotation(
          Line(points = {{10, 24}, {100, 24}, {100, 100}}, color = {0, 0, 127}));
        connect(idealGNSS.pos, bus.x_meas) annotation(
          Line(points = {{12, 4}, {100, 4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGNSS.vel, bus.v_meas) annotation(
          Line(points = {{12, -4}, {100, -4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGyroscope.w, bus.w_est) annotation(
          Line(points = {{10, 90}, {100, 90}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGNSS.pos, bus.x_est) annotation(
          Line(points = {{12, 4}, {100, 4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGNSS.vel, bus.v_est) annotation(
          Line(points = {{12, -4}, {100, -4}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(frame_a, quat_true.frame_a) annotation(
          Line(points = {{-100, 0}, {-40, 0}, {-40, -30}, {-8, -30}}));
        connect(quat_true.q, bus.q_est) annotation(
          Line(points = {{14, -30}, {100, -30}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(quaternion2Euler.q, quat_true.q) annotation(
          Line(points = {{54, -68}, {30, -68}, {30, -30}, {14, -30}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealAccelerometer.acc, bus.a_est) annotation(
          Line(points = {{10, 70}, {100, 70}, {100, 100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(eulerRates.q, quat_true.q) annotation(
          Line(points = {{12, -90}, {12, -60}, {14, -60}, {14, -30}}, color = {0, 0, 127}, thickness = 0.5));
        connect(idealGyroscope.w, eulerRates.w) annotation(
          Line(points = {{10, 90}, {12, 90}, {12, -78}}, color = {0, 0, 127}, thickness = 0.5));
      protected
        annotation(
          Icon(graphics = {Line(origin = {-83, 0}, points = {{-13, 0}, {13, 0}}), Text(origin = {-1.42109e-14, -40}, extent = {{-60, 20}, {60, -20}}, textString = "sens"), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name"), Line(origin = {85, 0}, points = {{-15, 0}, {9, 0}, {15, 0}})}));
      end SensorsIdealCont;

      model ControllersLQI
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.SystemMatrices systemMatrices(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-50, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 8, val = diagonal({0, 0, 0, 0, 0, 100, 1000, 1000})) annotation(
          Placement(visible = true, transformation(origin = {-50, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100} * 0.1)) annotation(
          Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, n = 8, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate(n1 = 2) annotation(
          Placement(visible = true, transformation(origin = {-50, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {150, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-91, 39}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-73, 39}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body annotation(
          Placement(visible = true, transformation(origin = {-16, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {62, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {84, 6.66134e-16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        GNC.Control.ErrorState errorState(m = 3, n = 5, p = 3) annotation(
          Placement(visible = true, transformation(origin = {-6, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant C(m = 5, n = 3, val = [0, 0, 1, 0, 0; 0, 0, 0, 1, 0; 0, 0, 0, 0, 1]) annotation(
          Placement(visible = true, transformation(origin = {-50, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant v_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-47, -87}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant w_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-191, -107}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant p_ref(k = from_deg(5)) annotation(
          Placement(visible = true, transformation(origin = {-191, -125}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Routing.Multiplex err_vec(n = 3) annotation(
          Placement(visible = true, transformation(origin = {72, -124}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback p_err annotation(
          Placement(visible = true, transformation(origin = {-13, -87}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback q_err annotation(
          Placement(visible = true, transformation(origin = {-13, -107}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback r_err annotation(
          Placement(visible = true, transformation(origin = {-13, -127}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 5) annotation(
          Placement(visible = true, transformation(origin = {74, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator p_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -87}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator q_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -107}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator r_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -127}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.GNC.Control.AngularRateToTarget angularRateToTarget(k = 1) annotation(
          Placement(visible = true, transformation(origin = {-62, -114}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Math.Blocks.Vector.VectorConstant vectorConstant1(k = {1, 0, 0}) annotation(
          Placement(visible = true, transformation(origin = {-152, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.ProjectOnPlane projectOnPlane(n = {0, 0, 1}) annotation(
          Placement(visible = true, transformation(origin = {-130, -108}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, vectorConcatenate.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -68}, {-62, -68}}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatrices.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 56}, {-62, 56}}, thickness = 0.5));
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 39}, {-95, 39}}, color = {0, 0, 127}));
        connect(density.rho, systemMatrices.rho) annotation(
          Line(points = {{-67.5, 39}, {-62, 39}, {-62, 50}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-88, 39}, {-79, 39}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, ned2body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 86}, {-4, 86}}, thickness = 0.5));
        connect(avionicsBus.v_est, ned2body.x_w) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 74}, {-4, 74}}, thickness = 0.5));
        connect(ned2body.x_b, systemMatrices.vel) annotation(
          Line(points = {{-26, 80}, {-84, 80}, {-84, 62}, {-62, 62}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQR.enable) annotation(
          Line(points = {{100, 100}, {36, 100}, {36, 9}}, color = {255, 0, 255}));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{73, 0}, {77, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(continuousLQR.u, control2Deflection.u) annotation(
          Line(points = {{47, 0}, {50, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ned2body.x_b[2:3], vectorConcatenate.v1) annotation(
          Line(points = {{-26, 80}, {-84, 80}, {-84, -60}, {-62, -60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Aaug, continuousLQR.A) annotation(
          Line(points = {{6, 40}, {20, 40}, {20, 8}, {24, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Baug, continuousLQR.B) annotation(
          Line(points = {{6, 30}, {18, 30}, {18, 4}, {24, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.A, errorState.A) annotation(
          Line(points = {{-38, 62}, {-30, 62}, {-30, 40}, {-18, 40}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.B, errorState.B) annotation(
          Line(points = {{-38, 52}, {-34, 52}, {-34, 34}, {-18, 34}}, color = {0, 0, 127}, thickness = 0.5));
        connect(C.k, errorState.C) annotation(
          Line(points = {{-38, 16}, {-30, 16}, {-30, 28}, {-18, 28}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{-38, -12}, {2, -12}, {2, 0}, {24, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{-38, -40}, {6, -40}, {6, -4}, {24, -4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_ref.y, p_err.u1) annotation(
          Line(points = {{-39, -87}, {-32, -87}, {-32, -86}, {-18, -86}}, color = {0, 0, 127}));
        connect(vectorConcatenate.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-38, -64}, {7, -64}, {7, -56}, {62, -56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(p_err.y, p_err_int.u) annotation(
          Line(points = {{-6, -86}, {14, -86}}, color = {0, 0, 127}));
        connect(q_err.y, q_err_int.u) annotation(
          Line(points = {{-6, -106}, {14, -106}}, color = {0, 0, 127}));
        connect(r_err.y, r_err_int.u) annotation(
          Line(points = {{-6, -126}, {14, -126}}, color = {0, 0, 127}));
        connect(p_err_int.y, err_vec.u[1]) annotation(
          Line(points = {{24, -86}, {50, -86}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(q_err_int.y, err_vec.u[2]) annotation(
          Line(points = {{24, -106}, {50, -106}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(r_err_int.y, err_vec.u[3]) annotation(
          Line(points = {{24, -126}, {50, -126}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{86, -60}, {96, -60}, {96, -26}, {16, -26}, {16, -8}, {24, -8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, r_err_int.reset) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -142}, {22, -142}, {22, -132}}, color = {255, 0, 255}));
        connect(r_err_int.reset, q_err_int.reset) annotation(
          Line(points = {{22, -132}, {22, -112}}, color = {255, 0, 255}));
        connect(q_err_int.reset, p_err_int.reset) annotation(
          Line(points = {{22, -112}, {22, -92}}, color = {255, 0, 255}));
        connect(err_vec.y, vectorConcatenate1.v2) annotation(
          Line(points = {{84, -124}, {88, -124}, {88, -78}, {38, -78}, {38, -64}, {62, -64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est[1], p_err.u2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -142}, {-12, -142}, {-12, -92}}, color = {0, 0, 127}));
        connect(avionicsBus.w_est[2], q_err.u2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -142}, {-14, -142}, {-14, -112}, {-12, -112}}, color = {0, 0, 127}));
        connect(avionicsBus.w_est[3], r_err.u2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -142}, {-12, -142}, {-12, -132}}, color = {0, 0, 127}));
        connect(angularRateToTarget.qr[1], q_err.u1) annotation(
          Line(points = {{-50, -114}, {-38, -114}, {-38, -106}, {-18, -106}}, color = {0, 0, 127}));
        connect(angularRateToTarget.qr[2], r_err.u1) annotation(
          Line(points = {{-50, -114}, {-38, -114}, {-38, -126}, {-18, -126}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, angularRateToTarget.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -120}, {-74, -120}}, thickness = 0.5));
        connect(vectorConstant1.v, angularRateToTarget.vt_ned) annotation(
          Line(points = {{-140, -66}, {-112, -66}, {-112, -114}, {-74, -114}}, color = {0, 0, 127}, thickness = 0.5));
        connect(projectOnPlane.vp, angularRateToTarget.v_ned) annotation(
          Line(points = {{-119, -108}, {-74, -108}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.v_est, projectOnPlane.v) annotation(
          Line(points = {{100, 100}, {-168, 100}, {-168, -108}, {-142, -108}}, thickness = 0.5));
        connect(finSaturation.fin_sat, fin) annotation(
          Line(points = {{90, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersLQI;

      model ControllersLQIHeading
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.SystemMatrices systemMatrices(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-50, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 7, val = diagonal({0, 0, 0, 0, 0, 1000, 1000})) annotation(
          Placement(visible = true, transformation(origin = {-50, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100} * 0.1)) annotation(
          Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, n = 7, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_base(n1 = 2) annotation(
          Placement(visible = true, transformation(origin = {-70, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {150, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-91, 39}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-73, 39}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body annotation(
          Placement(visible = true, transformation(origin = {-16, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {62, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {84, 6.66134e-16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        GNC.Control.ErrorState errorState(m = 3, n = 5, p = 2) annotation(
          Placement(visible = true, transformation(origin = {-6, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant v_ref(k = from_deg(-72)) annotation(
          Placement(visible = true, transformation(origin = {-47, -85}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant p_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-191, -125}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Routing.Multiplex err_vec(n = 2) annotation(
          Placement(visible = true, transformation(origin = {72, -124}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback roll_err annotation(
          Placement(visible = true, transformation(origin = {-13, -87}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback yaw_err annotation(
          Placement(visible = true, transformation(origin = {-13, -107}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 5, n2 = 2) annotation(
          Placement(visible = true, transformation(origin = {74, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator roll_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -87}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator yaw_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -107}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.RollHeadingOutput rollHeadingOutput annotation(
          Placement(visible = true, transformation(origin = {-50, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Blocks.EulerRates eulerRates annotation(
          Placement(visible = true, transformation(origin = {-120, -126}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ProportionalHeadingRate proportionalHeadingRate(k = 1 / 5) annotation(
          Placement(visible = true, transformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, x_base.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -68}, {-82, -68}}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatrices.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 56}, {-62, 56}}, thickness = 0.5));
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 39}, {-95, 39}}, color = {0, 0, 127}));
        connect(density.rho, systemMatrices.rho) annotation(
          Line(points = {{-67.5, 39}, {-62, 39}, {-62, 50}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-88, 39}, {-79, 39}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, ned2body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 86}, {-4, 86}}, thickness = 0.5));
        connect(avionicsBus.v_est, ned2body.x_w) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 74}, {-4, 74}}, thickness = 0.5));
        connect(ned2body.x_b, systemMatrices.vel) annotation(
          Line(points = {{-26, 80}, {-84, 80}, {-84, 62}, {-62, 62}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQR.enable) annotation(
          Line(points = {{100, 100}, {36, 100}, {36, 9}}, color = {255, 0, 255}));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{73, 0}, {77, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(continuousLQR.u, control2Deflection.u) annotation(
          Line(points = {{47, 0}, {50, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ned2body.x_b[2:3], x_base.v1) annotation(
          Line(points = {{-26, 80}, {-82, 80}, {-82, -60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Aaug, continuousLQR.A) annotation(
          Line(points = {{6, 40}, {20, 40}, {20, 8}, {24, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Baug, continuousLQR.B) annotation(
          Line(points = {{6, 30}, {18, 30}, {18, 4}, {24, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.A, errorState.A) annotation(
          Line(points = {{-38, 62}, {-30, 62}, {-30, 40}, {-18, 40}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.B, errorState.B) annotation(
          Line(points = {{-38, 52}, {-34, 52}, {-34, 34}, {-18, 34}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{-38, -12}, {2, -12}, {2, 0}, {24, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{-38, -40}, {6, -40}, {6, -4}, {24, -4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_ref.y, roll_err.u1) annotation(
          Line(points = {{-39, -85}, {-32, -85}, {-32, -86}, {-18, -86}}, color = {0, 0, 127}));
        connect(x_base.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-59, -64}, {7, -64}, {7, -56}, {62, -56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(roll_err.y, roll_err_int.u) annotation(
          Line(points = {{-6, -86}, {14, -86}}, color = {0, 0, 127}));
        connect(yaw_err.y, yaw_err_int.u) annotation(
          Line(points = {{-6, -106}, {14, -106}}, color = {0, 0, 127}));
        connect(roll_err_int.y, err_vec.u[1]) annotation(
          Line(points = {{24, -86}, {50, -86}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(yaw_err_int.y, err_vec.u[2]) annotation(
          Line(points = {{24, -106}, {50, -106}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{86, -60}, {96, -60}, {96, -26}, {16, -26}, {16, -8}, {24, -8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(yaw_err_int.reset, roll_err_int.reset) annotation(
          Line(points = {{22, -112}, {22, -92}}, color = {255, 0, 255}));
        connect(err_vec.y, vectorConcatenate1.v2) annotation(
          Line(points = {{84, -124}, {88, -124}, {88, -78}, {38, -78}, {38, -64}, {62, -64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(rollHeadingOutput.C, errorState.C) annotation(
          Line(points = {{-38, 18}, {-26, 18}, {-26, 28}, {-18, 28}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, rollHeadingOutput.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 18}, {-62, 18}}, thickness = 0.5));
        connect(avionicsBus.liftoff, yaw_err_int.reset) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -138}, {22, -138}, {22, -112}}, color = {255, 0, 255}));
        connect(finSaturation.fin_sat, fin) annotation(
          Line(points = {{90, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(eulerRates.euler_rates[3], roll_err.u2) annotation(
          Line(points = {{-109, -126}, {-12, -126}, {-12, -92}}, color = {0, 0, 127}));
        connect(eulerRates.euler_rates[1], yaw_err.u2) annotation(
          Line(points = {{-109, -126}, {-12, -126}, {-12, -112}}, color = {0, 0, 127}));
        connect(avionicsBus.w_est, eulerRates.w) annotation(
          Line(points = {{100, 100}, {-152, 100}, {-152, -120}, {-132, -120}}, thickness = 0.5));
        connect(avionicsBus.q_est, eulerRates.q) annotation(
          Line(points = {{100, 100}, {-152, 100}, {-152, -132}, {-132, -132}}, thickness = 0.5));
        connect(proportionalHeadingRate.hr, yaw_err.u1) annotation(
          Line(points = {{-39, -110}, {-18, -110}, {-18, -106}}, color = {0, 0, 127}));
        connect(p_ref.y, proportionalHeadingRate.track_ref) annotation(
          Line(points = {{-184, -124}, {-62, -124}, {-62, -116}}, color = {0, 0, 127}));
        connect(avionicsBus.v_est, proportionalHeadingRate.v_ned) annotation(
          Line(points = {{100, 100}, {-150, 100}, {-150, -104}, {-62, -104}}, thickness = 0.5));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersLQIHeading;

      model ControllersLQIHeadingFF
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.SystemMatrices systemMatrices(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-50, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 5, val = diagonal({10, 10, 1 / 3.14 * 10, 1 / 3.14 * 10, 1 / 3.14 * 10})) annotation(
          Placement(visible = true, transformation(origin = {-50, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100} * 0.01)) annotation(
          Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_base(n1 = 2) annotation(
          Placement(visible = true, transformation(origin = {-70, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {150, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-91, 39}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-73, 39}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body annotation(
          Placement(visible = true, transformation(origin = {-16, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {62, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {84, 6.66134e-16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant v_ref(k = from_deg(-10)) annotation(
          Placement(visible = true, transformation(origin = {-47, -85}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant p_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-191, -125}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.RollHeadingOutput rollHeadingOutput annotation(
          Placement(visible = true, transformation(origin = {-50, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ProportionalHeadingRate proportionalHeadingRate(k = 1 / 5) annotation(
          Placement(visible = true, transformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQRWithFF continuousLQRWithFF(m = 3, n = 5, p = 2, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, x_base.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -68}, {-82, -68}}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatrices.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 56}, {-62, 56}}, thickness = 0.5));
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 39}, {-95, 39}}, color = {0, 0, 127}));
        connect(density.rho, systemMatrices.rho) annotation(
          Line(points = {{-67.5, 39}, {-62, 39}, {-62, 50}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-88, 39}, {-79, 39}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, ned2body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 86}, {-4, 86}}, thickness = 0.5));
        connect(avionicsBus.v_est, ned2body.x_w) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 74}, {-4, 74}}, thickness = 0.5));
        connect(ned2body.x_b, systemMatrices.vel) annotation(
          Line(points = {{-26, 80}, {-84, 80}, {-84, 62}, {-62, 62}}, color = {0, 0, 127}, thickness = 0.5));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{73, 0}, {77, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ned2body.x_b[2:3], x_base.v1) annotation(
          Line(points = {{-26, 80}, {-82, 80}, {-82, -60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, rollHeadingOutput.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 18}, {-62, 18}}, thickness = 0.5));
        connect(finSaturation.fin_sat, fin) annotation(
          Line(points = {{90, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(p_ref.y, proportionalHeadingRate.track_ref) annotation(
          Line(points = {{-184, -124}, {-62, -124}, {-62, -116}}, color = {0, 0, 127}));
        connect(avionicsBus.v_est, proportionalHeadingRate.v_ned) annotation(
          Line(points = {{100, 100}, {-150, 100}, {-150, -104}, {-62, -104}}, thickness = 0.5));
        connect(continuousLQRWithFF.u, control2Deflection.u) annotation(
          Line(points = {{42, 0}, {50, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQRWithFF.enable) annotation(
          Line(points = {{100, 100}, {30, 100}, {30, 10}}, color = {255, 0, 255}));
        connect(systemMatrices.A, continuousLQRWithFF.A) annotation(
          Line(points = {{-38, 62}, {-8, 62}, {-8, 12}, {18, 12}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.B, continuousLQRWithFF.B) annotation(
          Line(points = {{-38, 52}, {-24, 52}, {-24, 8}, {18, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(rollHeadingOutput.C, continuousLQRWithFF.C) annotation(
          Line(points = {{-38, 18}, {-26, 18}, {-26, 4}, {18, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQRWithFF.Q) annotation(
          Line(points = {{-38, -12}, {-4, -12}, {-4, 0}, {18, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQRWithFF.R) annotation(
          Line(points = {{-38, -40}, {0, -40}, {0, -4}, {18, -4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(x_base.vc, continuousLQRWithFF.x) annotation(
          Line(points = {{-58, -64}, {4, -64}, {4, -8}, {18, -8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_ref.y, continuousLQRWithFF.r[1]) annotation(
          Line(points = {{-40, -84}, {10, -84}, {10, -12}, {18, -12}}, color = {0, 0, 127}));
        connect(proportionalHeadingRate.hr, continuousLQRWithFF.r[2]) annotation(
          Line(points = {{-38, -110}, {10, -110}, {10, -12}, {18, -12}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}, coordinateSystem(grid = {2, 0})));
      end ControllersLQIHeadingFF;

      model ControllersLQIHeadingFFInt
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.SystemMatrices systemMatrices(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-50, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 7, val = diagonal({10, 10, 10, 10, 10, 100, 100})) annotation(
          Placement(visible = true, transformation(origin = {-50, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100} * 0.1)) annotation(
          Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_base(n1 = 2) annotation(
          Placement(visible = true, transformation(origin = {-70, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {150, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-91, 39}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-73, 39}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body annotation(
          Placement(visible = true, transformation(origin = {-16, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {62, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {84, 6.66134e-16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        GNC.Control.ErrorState errorState(m = 3, n = 5, p = 2) annotation(
          Placement(visible = true, transformation(origin = {-6, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant v_ref(k = from_deg(-72)) annotation(
          Placement(visible = true, transformation(origin = {-47, -85}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant p_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-191, -125}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Routing.Multiplex err_vec(n = 2) annotation(
          Placement(visible = true, transformation(origin = {72, -124}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback roll_err annotation(
          Placement(visible = true, transformation(origin = {-13, -87}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback yaw_err annotation(
          Placement(visible = true, transformation(origin = {-13, -107}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 5, n2 = 2) annotation(
          Placement(visible = true, transformation(origin = {74, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator roll_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -87}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator yaw_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -107}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.RollHeadingOutput rollHeadingOutput annotation(
          Placement(visible = true, transformation(origin = {-50, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Blocks.EulerRates eulerRates annotation(
          Placement(visible = true, transformation(origin = {-120, -126}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ProportionalHeadingRate proportionalHeadingRate(k = 1 / 5) annotation(
          Placement(visible = true, transformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQRWithFF continuousLQRWithFF(m = 3, n = 7, p = 2, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {32, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, x_base.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -68}, {-82, -68}}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatrices.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 56}, {-62, 56}}, thickness = 0.5));
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 39}, {-95, 39}}, color = {0, 0, 127}));
        connect(density.rho, systemMatrices.rho) annotation(
          Line(points = {{-67.5, 39}, {-62, 39}, {-62, 50}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-88, 39}, {-79, 39}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, ned2body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 86}, {-4, 86}}, thickness = 0.5));
        connect(avionicsBus.v_est, ned2body.x_w) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 74}, {-4, 74}}, thickness = 0.5));
        connect(ned2body.x_b, systemMatrices.vel) annotation(
          Line(points = {{-26, 80}, {-84, 80}, {-84, 62}, {-62, 62}}, color = {0, 0, 127}, thickness = 0.5));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{73, 0}, {77, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ned2body.x_b[2:3], x_base.v1) annotation(
          Line(points = {{-26, 80}, {-82, 80}, {-82, -60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.A, errorState.A) annotation(
          Line(points = {{-38, 62}, {-30, 62}, {-30, 40}, {-18, 40}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.B, errorState.B) annotation(
          Line(points = {{-38, 52}, {-34, 52}, {-34, 34}, {-18, 34}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_ref.y, roll_err.u1) annotation(
          Line(points = {{-39, -85}, {-32, -85}, {-32, -86}, {-18, -86}}, color = {0, 0, 127}));
        connect(x_base.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-59, -64}, {7, -64}, {7, -56}, {62, -56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(roll_err.y, roll_err_int.u) annotation(
          Line(points = {{-6, -86}, {14, -86}}, color = {0, 0, 127}));
        connect(yaw_err.y, yaw_err_int.u) annotation(
          Line(points = {{-6, -106}, {14, -106}}, color = {0, 0, 127}));
        connect(roll_err_int.y, err_vec.u[1]) annotation(
          Line(points = {{24, -86}, {50, -86}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(yaw_err_int.y, err_vec.u[2]) annotation(
          Line(points = {{24, -106}, {50, -106}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(yaw_err_int.reset, roll_err_int.reset) annotation(
          Line(points = {{22, -112}, {22, -92}}, color = {255, 0, 255}));
        connect(err_vec.y, vectorConcatenate1.v2) annotation(
          Line(points = {{84, -124}, {88, -124}, {88, -78}, {38, -78}, {38, -64}, {62, -64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(rollHeadingOutput.C, errorState.C) annotation(
          Line(points = {{-38, 18}, {-26, 18}, {-26, 28}, {-18, 28}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, rollHeadingOutput.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 18}, {-62, 18}}, thickness = 0.5));
        connect(avionicsBus.liftoff, yaw_err_int.reset) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -138}, {22, -138}, {22, -112}}, color = {255, 0, 255}));
        connect(finSaturation.fin_sat, fin) annotation(
          Line(points = {{90, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(eulerRates.euler_rates[3], roll_err.u2) annotation(
          Line(points = {{-109, -126}, {-12, -126}, {-12, -92}}, color = {0, 0, 127}));
        connect(eulerRates.euler_rates[1], yaw_err.u2) annotation(
          Line(points = {{-109, -126}, {-12, -126}, {-12, -112}}, color = {0, 0, 127}));
        connect(avionicsBus.w_est, eulerRates.w) annotation(
          Line(points = {{100, 100}, {-152, 100}, {-152, -120}, {-132, -120}}, thickness = 0.5));
        connect(avionicsBus.q_est, eulerRates.q) annotation(
          Line(points = {{100, 100}, {-152, 100}, {-152, -132}, {-132, -132}}, thickness = 0.5));
        connect(proportionalHeadingRate.hr, yaw_err.u1) annotation(
          Line(points = {{-39, -110}, {-18, -110}, {-18, -106}}, color = {0, 0, 127}));
        connect(p_ref.y, proportionalHeadingRate.track_ref) annotation(
          Line(points = {{-184, -124}, {-62, -124}, {-62, -116}}, color = {0, 0, 127}));
        connect(avionicsBus.v_est, proportionalHeadingRate.v_ned) annotation(
          Line(points = {{100, 100}, {-150, 100}, {-150, -104}, {-62, -104}}, thickness = 0.5));
        connect(continuousLQRWithFF.u, control2Deflection.u) annotation(
          Line(points = {{44, 0}, {50, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Aaug, continuousLQRWithFF.A) annotation(
          Line(points = {{6, 40}, {12, 40}, {12, 12}, {20, 12}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Baug, continuousLQRWithFF.B) annotation(
          Line(points = {{6, 30}, {8, 30}, {8, 8}, {20, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Caug, continuousLQRWithFF.C) annotation(
          Line(points = {{6, 26}, {6, 4}, {20, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQRWithFF.Q) annotation(
          Line(points = {{-38, -12}, {10, -12}, {10, 0}, {20, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQRWithFF.R) annotation(
          Line(points = {{-38, -40}, {-8, -40}, {-8, -4}, {20, -4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate1.vc, continuousLQRWithFF.x) annotation(
          Line(points = {{86, -60}, {4, -60}, {4, -8}, {20, -8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_ref.y, continuousLQRWithFF.r[1]) annotation(
          Line(points = {{-40, -84}, {8, -84}, {8, -12}, {20, -12}}, color = {0, 0, 127}));
        connect(proportionalHeadingRate.hr, continuousLQRWithFF.r[2]) annotation(
          Line(points = {{-38, -110}, {8, -110}, {8, -12}, {20, -12}}, color = {0, 0, 127}));
        connect(avionicsBus.liftoff, continuousLQRWithFF.enable) annotation(
          Line(points = {{100, 100}, {32, 100}, {32, 10}}, color = {255, 0, 255}));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}, coordinateSystem(grid = {2, 0})));
      end ControllersLQIHeadingFFInt;

      model ControllersLQIHeadingFFInt2
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {178, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.SystemMatrices systemMatrices(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-50, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 7, val = diagonal({0, 0, 1, 1, 1, 10000, 10000})) annotation(
          Placement(visible = true, transformation(origin = {-50, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100} * 0.1)) annotation(
          Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, n = 7, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_base(n1 = 2) annotation(
          Placement(visible = true, transformation(origin = {-70, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {150, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-91, 39}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-73, 39}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body annotation(
          Placement(visible = true, transformation(origin = {-16, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {130, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {152, 6.66134e-16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        GNC.Control.ErrorState errorState(m = 3, n = 5, p = 2) annotation(
          Placement(visible = true, transformation(origin = {-6, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant v_ref(k = from_deg(-72)) annotation(
          Placement(visible = true, transformation(origin = {-47, -85}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant p_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-191, -125}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Routing.Multiplex err_vec(n = 2) annotation(
          Placement(visible = true, transformation(origin = {72, -124}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback roll_err annotation(
          Placement(visible = true, transformation(origin = {-13, -87}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback yaw_err annotation(
          Placement(visible = true, transformation(origin = {-13, -107}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 5, n2 = 2) annotation(
          Placement(visible = true, transformation(origin = {74, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator roll_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -87}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator yaw_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -107}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.RollHeadingOutput rollHeadingOutput annotation(
          Placement(visible = true, transformation(origin = {-50, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Blocks.EulerRates eulerRates annotation(
          Placement(visible = true, transformation(origin = {-120, -126}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ProportionalHeadingRate proportionalHeadingRate(k = 1 / 5) annotation(
          Placement(visible = true, transformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearFeedforward linearFeedforward(m = 3, n = 5, nk = 7, p = 2) annotation(
          Placement(visible = true, transformation(origin = {78, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, x_base.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -68}, {-82, -68}}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatrices.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 56}, {-62, 56}}, thickness = 0.5));
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 39}, {-95, 39}}, color = {0, 0, 127}));
        connect(density.rho, systemMatrices.rho) annotation(
          Line(points = {{-67.5, 39}, {-62, 39}, {-62, 50}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-88, 39}, {-79, 39}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, ned2body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 86}, {-4, 86}}, thickness = 0.5));
        connect(avionicsBus.v_est, ned2body.x_w) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 74}, {-4, 74}}, thickness = 0.5));
        connect(ned2body.x_b, systemMatrices.vel) annotation(
          Line(points = {{-26, 80}, {-84, 80}, {-84, 62}, {-62, 62}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQR.enable) annotation(
          Line(points = {{100, 100}, {36, 100}, {36, 9}}, color = {255, 0, 255}));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{141, 0}, {145, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ned2body.x_b[2:3], x_base.v1) annotation(
          Line(points = {{-26, 80}, {-82, 80}, {-82, -60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Aaug, continuousLQR.A) annotation(
          Line(points = {{6, 40}, {20, 40}, {20, 8}, {24, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Baug, continuousLQR.B) annotation(
          Line(points = {{6, 30}, {18, 30}, {18, 4}, {24, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.A, errorState.A) annotation(
          Line(points = {{-38, 62}, {-30, 62}, {-30, 40}, {-18, 40}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.B, errorState.B) annotation(
          Line(points = {{-38, 52}, {-34, 52}, {-34, 34}, {-18, 34}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{-38, -12}, {2, -12}, {2, 0}, {24, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{-38, -40}, {6, -40}, {6, -4}, {24, -4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_ref.y, roll_err.u1) annotation(
          Line(points = {{-39, -85}, {-32, -85}, {-32, -86}, {-18, -86}}, color = {0, 0, 127}));
        connect(x_base.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-59, -64}, {7, -64}, {7, -56}, {62, -56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(roll_err.y, roll_err_int.u) annotation(
          Line(points = {{-6, -86}, {14, -86}}, color = {0, 0, 127}));
        connect(yaw_err.y, yaw_err_int.u) annotation(
          Line(points = {{-6, -106}, {14, -106}}, color = {0, 0, 127}));
        connect(roll_err_int.y, err_vec.u[1]) annotation(
          Line(points = {{24, -86}, {50, -86}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(yaw_err_int.y, err_vec.u[2]) annotation(
          Line(points = {{24, -106}, {50, -106}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{86, -60}, {96, -60}, {96, -26}, {16, -26}, {16, -8}, {24, -8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(yaw_err_int.reset, roll_err_int.reset) annotation(
          Line(points = {{22, -112}, {22, -92}}, color = {255, 0, 255}));
        connect(err_vec.y, vectorConcatenate1.v2) annotation(
          Line(points = {{84, -124}, {88, -124}, {88, -78}, {38, -78}, {38, -64}, {62, -64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(rollHeadingOutput.C, errorState.C) annotation(
          Line(points = {{-38, 18}, {-26, 18}, {-26, 28}, {-18, 28}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, rollHeadingOutput.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 18}, {-62, 18}}, thickness = 0.5));
        connect(avionicsBus.liftoff, yaw_err_int.reset) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -138}, {22, -138}, {22, -112}}, color = {255, 0, 255}));
        connect(finSaturation.fin_sat, fin) annotation(
          Line(points = {{158.6, 6.66134e-16}, {178.6, 6.66134e-16}}, color = {0, 0, 127}, thickness = 0.5));
        connect(eulerRates.euler_rates[3], roll_err.u2) annotation(
          Line(points = {{-109, -126}, {-12, -126}, {-12, -92}}, color = {0, 0, 127}));
        connect(eulerRates.euler_rates[1], yaw_err.u2) annotation(
          Line(points = {{-109, -126}, {-12, -126}, {-12, -112}}, color = {0, 0, 127}));
        connect(avionicsBus.w_est, eulerRates.w) annotation(
          Line(points = {{100, 100}, {-152, 100}, {-152, -120}, {-132, -120}}, thickness = 0.5));
        connect(avionicsBus.q_est, eulerRates.q) annotation(
          Line(points = {{100, 100}, {-152, 100}, {-152, -132}, {-132, -132}}, thickness = 0.5));
        connect(proportionalHeadingRate.hr, yaw_err.u1) annotation(
          Line(points = {{-39, -110}, {-18, -110}, {-18, -106}}, color = {0, 0, 127}));
        connect(p_ref.y, proportionalHeadingRate.track_ref) annotation(
          Line(points = {{-184, -124}, {-62, -124}, {-62, -116}}, color = {0, 0, 127}));
        connect(avionicsBus.v_est, proportionalHeadingRate.v_ned) annotation(
          Line(points = {{100, 100}, {-150, 100}, {-150, -104}, {-62, -104}}, thickness = 0.5));
        connect(continuousLQR.u, linearFeedforward.u) annotation(
          Line(points = {{48, 0}, {66, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(continuousLQR.K, linearFeedforward.K) annotation(
          Line(points = {{48, 6}, {66, 6}, {66, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.A, linearFeedforward.A) annotation(
          Line(points = {{-38, 62}, {48, 62}, {48, 16}, {66, 16}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatrices.B, linearFeedforward.B) annotation(
          Line(points = {{-38, 52}, {48, 52}, {48, 12}, {66, 12}}, color = {0, 0, 127}, thickness = 0.5));
        connect(rollHeadingOutput.C, linearFeedforward.C) annotation(
          Line(points = {{-38, 18}, {66, 18}, {66, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_ref.y, linearFeedforward.r[1]) annotation(
          Line(points = {{-40, -84}, {66, -84}, {66, -4}}, color = {0, 0, 127}));
        connect(proportionalHeadingRate.hr, linearFeedforward.r[2]) annotation(
          Line(points = {{-38, -110}, {66, -110}, {66, -4}}, color = {0, 0, 127}));
        connect(linearFeedforward.u_ff, control2Deflection.u) annotation(
          Line(points = {{90, 2}, {118, 2}, {118, 0}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}, coordinateSystem(grid = {2, 0})));
      end ControllersLQIHeadingFFInt2;

      model ControllersLQIU
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {182, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 7, val = diagonal({0, 10, 10, 0, 0, 0, 1000})) annotation(
          Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100} * 0.1)) annotation(
          Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, maxiter = 50, n = 7, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_base(n1 = 3) annotation(
          Placement(visible = true, transformation(origin = {-74, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {150, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-91, 39}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-73, 39}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body annotation(
          Placement(visible = true, transformation(origin = {-16, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {112, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {134, 6.66134e-16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        GNC.Control.ErrorState errorState(m = 3, n = 6, p = 1, use_D = false) annotation(
          Placement(visible = true, transformation(origin = {-6, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant v_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-45, -87}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 6, n2 = 1) annotation(
          Placement(visible = true, transformation(origin = {74, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.SystemMatricesU systemMatricesU(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-44, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.RollOutput rollOutput annotation(
          Placement(visible = true, transformation(origin = {-46, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.AccelRef accelRef(k = 0.1) annotation(
          Placement(visible = true, transformation(origin = {-78, -104}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Math.Blocks.Vector.VectorConstant vectorConstant2(k = {0, 0, 9.81}) annotation(
          Placement(visible = true, transformation(origin = {-202, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body1 annotation(
          Placement(visible = true, transformation(origin = {-122, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FeedbackIntegrator feedbackIntegrator(n = 3) annotation(
          Placement(visible = true, transformation(origin = {-2, -88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorGain vectorGain1(external_gain = false, gain = 1, n = 2) annotation(
          Placement(visible = true, transformation(origin = {42, -106}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant const(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-175, -93}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        RocketControl.Components.Blocks.EulerRates eulerRates annotation(
          Placement(visible = true, transformation(origin = {-134, -134}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorAdd vectorAdd annotation(
          Placement(visible = true, transformation(origin = {-128, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, x_base.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -72}, {-86, -72}}, thickness = 0.5));
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 39}, {-95, 39}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-88, 39}, {-79, 39}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, ned2body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 86}, {-4, 86}}, thickness = 0.5));
        connect(avionicsBus.v_est, ned2body.x_w) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 74}, {-4, 74}}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQR.enable) annotation(
          Line(points = {{100, 100}, {36, 100}, {36, 9}}, color = {255, 0, 255}));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{123, 0}, {127, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(continuousLQR.u, control2Deflection.u) annotation(
          Line(points = {{47, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Aaug, continuousLQR.A) annotation(
          Line(points = {{6, 40}, {20, 40}, {20, 8}, {24, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Baug, continuousLQR.B) annotation(
          Line(points = {{6, 30}, {18, 30}, {18, 4}, {24, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{-39, -10}, {2, -10}, {2, 0}, {24, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{-38, -40}, {6, -40}, {6, -4}, {24, -4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{86, -60}, {96, -60}, {96, -26}, {16, -26}, {16, -8}, {24, -8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatricesU.A, errorState.A) annotation(
          Line(points = {{-32, 52}, {-24, 52}, {-24, 42}, {-18, 42}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatricesU.B, errorState.B) annotation(
          Line(points = {{-32, 42}, {-26, 42}, {-26, 38}, {-18, 38}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatricesU.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 46}, {-56, 46}}, thickness = 0.5));
        connect(density.rho, systemMatricesU.rho) annotation(
          Line(points = {{-68, 40}, {-56, 40}}, color = {0, 0, 127}));
        connect(ned2body.x_b, systemMatricesU.vel) annotation(
          Line(points = {{-26, 80}, {-84, 80}, {-84, 52}, {-56, 52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, rollOutput.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 18}, {-58, 18}}, thickness = 0.5));
        connect(rollOutput.C, errorState.C) annotation(
          Line(points = {{-34, 18}, {-24, 18}, {-24, 32}, {-18, 32}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ned2body.x_b[1], x_base.v1[1]) annotation(
          Line(points = {{-26, 80}, {-94, 80}, {-94, -64}, {-86, -64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(x_base.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-63, -68}, {-2.5, -68}, {-2.5, -56}, {62, -56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConstant2.v, ned2body1.x_w) annotation(
          Line(points = {{-190, 32}, {-156, 32}, {-156, 38}, {-134, 38}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, ned2body1.q_bw) annotation(
          Line(points = {{100, 100}, {-134, 100}, {-134, 26}}, thickness = 0.5));
        connect(v_ref.y, feedbackIntegrator.ref[1]) annotation(
          Line(points = {{-37, -87}, {-26, -87}, {-26, -88}, {-14, -88}}, color = {0, 0, 127}));
        connect(accelRef.a_body[2], feedbackIntegrator.ref[2]) annotation(
          Line(points = {{-66, -104}, {-26, -104}, {-26, -88}, {-14, -88}}, color = {0, 0, 127}, thickness = 0.5));
        connect(accelRef.a_body[3], feedbackIntegrator.ref[3]) annotation(
          Line(points = {{-66, -104}, {-26, -104}, {-26, -88}, {-14, -88}}, color = {0, 0, 127}, thickness = 0.5));
        connect(feedbackIntegrator.err_int[1], vectorConcatenate1.v2[1]) annotation(
          Line(points = {{10, -88}, {26, -88}, {26, -64}, {62, -64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(feedbackIntegrator.err_int[2:3], vectorGain1.v) annotation(
          Line(points = {{10, -88}, {18, -88}, {18, -100}, {30, -100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorGain1.vk, x_base.v1[2:3]) annotation(
          Line(points = {{54, -106}, {56, -106}, {56, -56}, {-94, -56}, {-94, -64}, {-86, -64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.v_est, accelRef.vel) annotation(
          Line(points = {{100, 100}, {-144, 100}, {-144, -106}, {-90, -106}}, thickness = 0.5));
        connect(avionicsBus.q_est, accelRef.q) annotation(
          Line(points = {{100, 100}, {-144, 100}, {-144, -112}, {-90, -112}}, thickness = 0.5));
        connect(const.y, accelRef.track_target) annotation(
          Line(points = {{-168, -92}, {-90, -92}, {-90, -98}}, color = {0, 0, 127}));
        connect(eulerRates.euler_rates[3], feedbackIntegrator.feedback[1]) annotation(
          Line(points = {{-122, -134}, {-2, -134}, {-2, -100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est, eulerRates.w) annotation(
          Line(points = {{100, 100}, {-218, 100}, {-218, -128}, {-146, -128}}, thickness = 0.5));
        connect(avionicsBus.q_est, eulerRates.q) annotation(
          Line(points = {{100, 100}, {-218, 100}, {-218, -140}, {-146, -140}}, thickness = 0.5));
        connect(ned2body1.x_b, vectorAdd.v2) annotation(
          Line(points = {{-110, 32}, {-108, 32}, {-108, -36}, {-158, -36}, {-158, -82}, {-140, -82}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.a_est, vectorAdd.v1) annotation(
          Line(points = {{100, 100}, {-162, 100}, {-162, -74}, {-140, -74}}, thickness = 0.5));
        connect(vectorAdd.vc[2], feedbackIntegrator.feedback[2]) annotation(
          Line(points = {{-116, -78}, {-108, -78}, {-108, -128}, {-2, -128}, {-2, -100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorAdd.vc[3], feedbackIntegrator.feedback[3]) annotation(
          Line(points = {{-116, -78}, {-108, -78}, {-108, -128}, {-2, -128}, {-2, -100}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConstant.v, fin) annotation(
          Line(points = {{162, -30}, {182, -30}, {182, 0}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersLQIU;

      model ControllersLQIder
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {166, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 17, val = diagonal({0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1000, 10})) annotation(
          Placement(visible = true, transformation(origin = {-50, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100} * 1)) annotation(
          Placement(visible = true, transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, maxiter = 100, n = 17, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_base annotation(
          Placement(visible = true, transformation(origin = {-84, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {150, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-91, 39}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-73, 39}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body annotation(
          Placement(visible = true, transformation(origin = {-16, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {118, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {140, 6.66134e-16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        GNC.Control.ErrorState errorState(m = 3, n = 15, p = 2) annotation(
          Placement(visible = true, transformation(origin = {-6, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant v_ref(k = from_deg(-72)) annotation(
          Placement(visible = true, transformation(origin = {-47, -103}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant p_ref(k = 0.5) annotation(
          Placement(visible = true, transformation(origin = {-191, -125}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Routing.Multiplex err_vec(n = 2) annotation(
          Placement(visible = true, transformation(origin = {72, -124}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback roll_err annotation(
          Placement(visible = true, transformation(origin = {-13, -87}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Math.Feedback vd_err annotation(
          Placement(visible = true, transformation(origin = {-1, -109}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 15, n2 = 2) annotation(
          Placement(visible = true, transformation(origin = {74, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator roll_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -87}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator vd_err_int(use_reset = true) annotation(
          Placement(visible = true, transformation(origin = {19, -109}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.EulerRates eulerRates annotation(
          Placement(visible = true, transformation(origin = {-118, -114}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.SystemMatricesDer systemMatricesDer(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-46, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        GNC.Control.LinearLQ.OutputMatrixDer outputMatrixDer annotation(
          Placement(visible = true, transformation(origin = {-38, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorIntegrate finctrl annotation(
          Placement(visible = true, transformation(origin = {76, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate annotation(
          Placement(visible = true, transformation(origin = {-84, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Derivative derivative[3] annotation(
          Placement(visible = true, transformation(origin = {-180, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate2(n1 = 6, n2 = 6) annotation(
          Placement(visible = true, transformation(origin = {-44, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate3(n1 = 12, n2 = 3) annotation(
          Placement(visible = true, transformation(origin = {8, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body1 annotation(
          Placement(visible = true, transformation(origin = {-58, 124}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.Math.Blocks.Vector.VectorAdd vectorAdd annotation(
          Placement(visible = true, transformation(origin = {-134, 136}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant1(k = {0, 0, 9.81}) annotation(
          Placement(visible = true, transformation(origin = {-10, 114}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      equation
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 39}, {-95, 39}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-88, 39}, {-79, 39}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, ned2body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 86}, {-4, 86}}, thickness = 0.5));
        connect(avionicsBus.v_est, ned2body.x_w) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 74}, {-4, 74}}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQR.enable) annotation(
          Line(points = {{100, 100}, {36, 100}, {36, 9}}, color = {255, 0, 255}));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{129, 0}, {133, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Aaug, continuousLQR.A) annotation(
          Line(points = {{6, 40}, {20, 40}, {20, 8}, {24, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Baug, continuousLQR.B) annotation(
          Line(points = {{6, 30}, {18, 30}, {18, 4}, {24, 4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{-38, -12}, {2, -12}, {2, 0}, {24, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{-38, -40}, {6, -40}, {6, -4}, {24, -4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_ref.y, roll_err.u1) annotation(
          Line(points = {{-39, -103}, {-32, -103}, {-32, -86}, {-18, -86}}, color = {0, 0, 127}));
        connect(roll_err.y, roll_err_int.u) annotation(
          Line(points = {{-6, -86}, {14, -86}}, color = {0, 0, 127}));
        connect(vd_err.y, vd_err_int.u) annotation(
          Line(points = {{5, -109}, {13, -109}}, color = {0, 0, 127}));
        connect(roll_err_int.y, err_vec.u[1]) annotation(
          Line(points = {{24, -86}, {50, -86}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(vd_err_int.y, err_vec.u[2]) annotation(
          Line(points = {{24.5, -109}, {50, -109}, {50, -124}, {62, -124}}, color = {0, 0, 127}));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{86, -60}, {96, -60}, {96, -26}, {16, -26}, {16, -8}, {24, -8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vd_err_int.reset, roll_err_int.reset) annotation(
          Line(points = {{22, -115}, {22, -92}}, color = {255, 0, 255}));
        connect(err_vec.y, vectorConcatenate1.v2) annotation(
          Line(points = {{84, -124}, {88, -124}, {88, -78}, {38, -78}, {38, -64}, {62, -64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(eulerRates.euler_rates[3], roll_err.u2) annotation(
          Line(points = {{-107, -114}, {-12, -114}, {-12, -92}}, color = {0, 0, 127}));
        connect(avionicsBus.w_est, eulerRates.w) annotation(
          Line(points = {{100, 100}, {-152, 100}, {-152, -108}, {-130, -108}}, thickness = 0.5));
        connect(avionicsBus.q_est, eulerRates.q) annotation(
          Line(points = {{100, 100}, {-152, 100}, {-152, -120}, {-130, -120}}, thickness = 0.5));
        connect(avionicsBus.q_est, outputMatrixDer.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 20}, {-50, 20}}, thickness = 0.5));
        connect(ned2body.x_b, systemMatricesDer.vel) annotation(
          Line(points = {{-26, 80}, {-82, 80}, {-82, 72}, {-58, 72}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatricesDer.ang_vel) annotation(
          Line(points = {{100, 100}, {-82, 100}, {-82, 68}, {-58, 68}}, thickness = 0.5));
        connect(density.rho, systemMatricesDer.rho) annotation(
          Line(points = {{-68, 40}, {-58, 40}, {-58, 54}}, color = {0, 0, 127}));
        connect(systemMatricesDer.A, errorState.A) annotation(
          Line(points = {{-35, 67}, {-24, 67}, {-24, 42}, {-18, 42}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatricesDer.B, errorState.B) annotation(
          Line(points = {{-35, 57}, {-26, 57}, {-26, 38}, {-18, 38}}, color = {0, 0, 127}, thickness = 0.5));
        connect(outputMatrixDer.C, errorState.C) annotation(
          Line(points = {{-26, 20}, {-24, 20}, {-24, 32}, {-18, 32}}, color = {0, 0, 127}, thickness = 0.5));
        connect(continuousLQR.u, finctrl.v) annotation(
          Line(points = {{48, 0}, {64, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(finctrl.vi, control2Deflection.u) annotation(
          Line(points = {{88, 0}, {106, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ned2body.x_b, x_base.v1) annotation(
          Line(points = {{-26, 80}, {-96, 80}, {-96, -60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est, vectorConcatenate.v1) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -86}, {-96, -86}}, thickness = 0.5));
        connect(avionicsBus.w_est, derivative.u) annotation(
          Line(points = {{100, 100}, {-204, 100}, {-204, -62}, {-192, -62}}, thickness = 0.5));
        connect(derivative.y, vectorConcatenate.v2) annotation(
          Line(points = {{-168, -62}, {-104, -62}, {-104, -94}, {-96, -94}}, color = {0, 0, 127}, thickness = 0.5));
        connect(x_base.vc, vectorConcatenate2.v1) annotation(
          Line(points = {{-72, -64}, {-65, -64}, {-65, -62}, {-56, -62}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate.vc, vectorConcatenate2.v2) annotation(
          Line(points = {{-72, -90}, {-56, -90}, {-56, -70}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate2.vc, vectorConcatenate3.v1) annotation(
          Line(points = {{-32, -66}, {-20, -66}, {-20, -60}, {-4, -60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(finctrl.vi, vectorConcatenate3.v2) annotation(
          Line(points = {{88, 0}, {-22, 0}, {-22, -68}, {-4, -68}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate3.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{19, -64}, {32, -64}, {32, -56}, {62, -56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConstant.v, fin) annotation(
          Line(points = {{162, -30}, {166, -30}, {166, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(p_ref.y, vd_err.u1) annotation(
          Line(points = {{-184, -124}, {-6, -124}, {-6, -108}}, color = {0, 0, 127}));
        connect(avionicsBus.a_est[2], vd_err.u2) annotation(
          Line(points = {{100, 100}, {-152, 100}, {-152, -146}, {0, -146}, {0, -114}}, color = {0, 0, 127}));
        connect(derivative.y, systemMatricesDer.ang_acc) annotation(
          Line(points = {{-168, -62}, {-134, -62}, {-134, 60}, {-58, 60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, vd_err_int.reset) annotation(
          Line(points = {{100, 100}, {22, 100}, {22, -114}}, color = {255, 0, 255}));
        connect(finctrl.vi, systemMatricesDer.delta) annotation(
          Line(points = {{88, 0}, {76, 0}, {76, 100}, {-82, 100}, {-82, 56}, {-58, 56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, ned2body1.q_bw) annotation(
          Line(points = {{100, 100}, {100, 130}, {-46, 130}}, thickness = 0.5));
        connect(avionicsBus.a_est, vectorAdd.v2) annotation(
          Line(points = {{100, 100}, {100, 140}, {-122, 140}}, thickness = 0.5));
        connect(ned2body1.x_b, vectorAdd.v1) annotation(
          Line(points = {{-68, 124}, {-122, 124}, {-122, 132}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorAdd.vc, systemMatricesDer.acc) annotation(
          Line(points = {{-144, 136}, {-152, 136}, {-152, 64}, {-58, 64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorAdd.vc, x_base.v2) annotation(
          Line(points = {{-144, 136}, {-152, 136}, {-152, -68}, {-96, -68}}, color = {0, 0, 127}, thickness = 0.5));
        connect(ned2body1.x_w, vectorConstant1.v) annotation(
          Line(points = {{-46, 118}, {-20, 118}, {-20, 114}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersLQIder;

      model ControllersLQIVned
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {190, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 13, val = diagonal({0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0.01})) annotation(
          Placement(visible = true, transformation(origin = {-8, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100000000} * 0.05)) annotation(
          Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, maxiter = 50, n = 13, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_vw(n1 = 3) annotation(
          Placement(visible = true, transformation(origin = {-78, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {140, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-93, 129}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        RocketControl.World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-75, 129}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body v_est_body annotation(
          Placement(visible = true, transformation(origin = {-18, 116}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {96, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {142, 52}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.GNC.Control.ErrorState errorState(m = 3, n = 10, p = 3, use_D = false) annotation(
          Placement(visible = true, transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 10, n2 = 3) annotation(
          Placement(visible = true, transformation(origin = {74, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        GNC.Control.LinearLQ.SystemMatricesQ systemMatricesQ(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-56, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.OutputNedVel outputNedVel annotation(
          Placement(visible = true, transformation(origin = {-56, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_base(n1 = 6, n2 = 4) annotation(
          Placement(visible = true, transformation(origin = {-42, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        GNC.Control.AccelRef accelRef(k = 0.1, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {-66, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant track_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-149, -73}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator integrator[3] annotation(
          Placement(visible = true, transformation(origin = {0, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, x_vw.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -24}, {-90, -24}}, thickness = 0.5));
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 129}, {-97, 129}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-89.7, 129}, {-80.7, 129}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, v_est_body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 122}, {-6, 122}}, thickness = 0.5));
        connect(avionicsBus.v_est, v_est_body.x_w) annotation(
          Line(points = {{100, 100}, {48, 100}, {48, 110}, {-6, 110}}, thickness = 0.5));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{107, 50}, {133, 50}, {133, 52}, {135, 52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(continuousLQR.u, control2Deflection.u) annotation(
          Line(points = {{61, 50}, {84, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Aaug, continuousLQR.A) annotation(
          Line(points = {{1, 75}, {20, 75}, {20, 58}, {38, 58}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Baug, continuousLQR.B) annotation(
          Line(points = {{1, 65}, {18, 65}, {18, 54}, {38, 54}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{3, 40}, {10.5, 40}, {10.5, 50}, {38, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{1, 10}, {5.5, 10}, {5.5, 46}, {38, 46}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{86, -60}, {96, -60}, {96, -26}, {16, -26}, {16, 42}, {38, 42}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_est_body.x_b, x_vw.v1) annotation(
          Line(points = {{-29, 116}, {-100, 116}, {-100, -16}, {-90, -16}}, color = {0, 0, 127}, thickness = 0.5));
        connect(finSaturation.fin_sat, fin) annotation(
          Line(points = {{148.6, 52}, {190.6, 52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatricesQ.vel, outputNedVel.vel) annotation(
          Line(points = {{-68, 88}, {-82, 88}, {-82, 52}, {-68, 52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatricesQ.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 82}, {-68, 82}}, thickness = 0.5));
        connect(avionicsBus.w_est, outputNedVel.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 46}, {-68, 46}}, thickness = 0.5));
        connect(avionicsBus.q_est, systemMatricesQ.quat) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 78}, {-68, 78}}, thickness = 0.5));
        connect(avionicsBus.q_est, outputNedVel.quat) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 38}, {-68, 38}}, thickness = 0.5));
        connect(density.rho, systemMatricesQ.rho) annotation(
          Line(points = {{-70, 130}, {-66, 130}, {-66, 98}, {-90, 98}, {-90, 72}, {-68, 72}}, color = {0, 0, 127}));
        connect(systemMatricesQ.A, errorState.A) annotation(
          Line(points = {{-44, 86}, {-32, 86}, {-32, 78}, {-22, 78}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatricesQ.B, errorState.B) annotation(
          Line(points = {{-44, 76}, {-34, 76}, {-34, 74}, {-22, 74}}, color = {0, 0, 127}, thickness = 0.5));
        connect(outputNedVel.C, errorState.C) annotation(
          Line(points = {{-44, 46}, {-36, 46}, {-36, 68}, {-22, 68}}, color = {0, 0, 127}, thickness = 0.5));
        connect(x_vw.vc, x_base.v1) annotation(
          Line(points = {{-66, -20}, {-62, -20}, {-62, -32}, {-54, -32}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, x_base.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -40}, {-54, -40}}, thickness = 0.5));
        connect(x_base.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-30, -36}, {30, -36}, {30, -56}, {62, -56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_est_body.x_b, systemMatricesQ.vel) annotation(
          Line(points = {{-28, 116}, {-100, 116}, {-100, 88}, {-68, 88}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQR.enable) annotation(
          Line(points = {{100, 100}, {50, 100}, {50, 60}}, color = {255, 0, 255}));
        connect(avionicsBus.liftoff, accelRef.enable) annotation(
          Line(points = {{100, 100}, {-66, 100}, {-66, -72}}, color = {255, 0, 255}));
        connect(avionicsBus.a_est, accelRef.acc_body) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -100}, {-66, -100}, {-66, -92}}, thickness = 0.5));
        connect(avionicsBus.v_est, accelRef.vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -84}, {-78, -84}}, thickness = 0.5));
        connect(avionicsBus.q_est, accelRef.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -90}, {-78, -90}}, thickness = 0.5));
        connect(track_ref.y, accelRef.track_target) annotation(
          Line(points = {{-142, -72}, {-96, -72}, {-96, -76}, {-78, -76}}, color = {0, 0, 127}));
        connect(accelRef.vel_out, integrator.u) annotation(
          Line(points = {{-54, -82}, {-12, -82}}, color = {0, 0, 127}, thickness = 0.5));
        connect(integrator.y, vectorConcatenate1.v2) annotation(
          Line(points = {{11, -82}, {36, -82}, {36, -64}, {62, -64}}, color = {0, 0, 127}, thickness = 0.5));
        connect(integrator.reset, accelRef.enable) annotation(
          Line(points = {{6, -94}, {6, -100}, {-44, -100}, {-44, -62}, {-66, -62}, {-66, -72}}, color = {255, 0, 255}));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersLQIVned;

      model ControllersLQIVnedSimpl
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {190, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 11, val = diagonal({0, 0, 0, 10, 0, 0, 0, 0, 0, 0.01, 0.01})) annotation(
          Placement(visible = true, transformation(origin = {-8, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100} * 0.05)) annotation(
          Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, maxiter = 50, n = 11, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_vw(n1 = 3) annotation(
          Placement(visible = true, transformation(origin = {-78, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {140, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-93, 129}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        RocketControl.World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-75, 129}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body v_est_body annotation(
          Placement(visible = true, transformation(origin = {-18, 116}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {96, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {142, 52}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.GNC.Control.ErrorState errorState(m = 3, n = 9, p = 2, use_D = false) annotation(
          Placement(visible = true, transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 9, n2 = 2) annotation(
          Placement(visible = true, transformation(origin = {74, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        GNC.Control.LinearLQ.SystemMatricesQSimpl systemMatricesQ(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-56, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.OutputNedVelSimpl outputNedVel annotation(
          Placement(visible = true, transformation(origin = {-56, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_base(n1 = 6, n2 = 3) annotation(
          Placement(visible = true, transformation(origin = {-42, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant1(k = {0, 0, 0}) annotation(
          Placement(visible = true, transformation(origin = {-140, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.AccelRef accelRef annotation(
          Placement(visible = true, transformation(origin = {-64, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant track_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-149, -73}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, x_vw.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -24}, {-90, -24}}, thickness = 0.5));
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 129}, {-97, 129}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-89.7, 129}, {-80.7, 129}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, v_est_body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 122}, {-6, 122}}, thickness = 0.5));
        connect(avionicsBus.v_est, v_est_body.x_w) annotation(
          Line(points = {{100, 100}, {48, 100}, {48, 110}, {-6, 110}}, thickness = 0.5));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{107, 50}, {133, 50}, {133, 52}, {135, 52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(continuousLQR.u, control2Deflection.u) annotation(
          Line(points = {{61, 50}, {84, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Aaug, continuousLQR.A) annotation(
          Line(points = {{1, 75}, {20, 75}, {20, 58}, {38, 58}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Baug, continuousLQR.B) annotation(
          Line(points = {{1, 65}, {18, 65}, {18, 54}, {38, 54}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{3, 40}, {10.5, 40}, {10.5, 50}, {38, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{1, 10}, {5.5, 10}, {5.5, 46}, {38, 46}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{86, -60}, {96, -60}, {96, -26}, {16, -26}, {16, 42}, {38, 42}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_est_body.x_b, x_vw.v1) annotation(
          Line(points = {{-29, 116}, {-100, 116}, {-100, -16}, {-90, -16}}, color = {0, 0, 127}, thickness = 0.5));
        connect(finSaturation.fin_sat, fin) annotation(
          Line(points = {{148.6, 52}, {190.6, 52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatricesQ.vel, outputNedVel.vel) annotation(
          Line(points = {{-68, 88}, {-82, 88}, {-82, 52}, {-68, 52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatricesQ.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 82}, {-68, 82}}, thickness = 0.5));
        connect(avionicsBus.w_est, outputNedVel.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 46}, {-68, 46}}, thickness = 0.5));
        connect(avionicsBus.q_est, systemMatricesQ.quat) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 78}, {-68, 78}}, thickness = 0.5));
        connect(avionicsBus.q_est, outputNedVel.quat) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 38}, {-68, 38}}, thickness = 0.5));
        connect(density.rho, systemMatricesQ.rho) annotation(
          Line(points = {{-70, 130}, {-66, 130}, {-66, 98}, {-90, 98}, {-90, 72}, {-68, 72}}, color = {0, 0, 127}));
        connect(systemMatricesQ.A, errorState.A) annotation(
          Line(points = {{-44, 86}, {-32, 86}, {-32, 78}, {-22, 78}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatricesQ.B, errorState.B) annotation(
          Line(points = {{-44, 76}, {-34, 76}, {-34, 74}, {-22, 74}}, color = {0, 0, 127}, thickness = 0.5));
        connect(outputNedVel.C, errorState.C) annotation(
          Line(points = {{-44, 46}, {-36, 46}, {-36, 68}, {-22, 68}}, color = {0, 0, 127}, thickness = 0.5));
        connect(x_vw.vc, x_base.v1) annotation(
          Line(points = {{-66, -20}, {-62, -20}, {-62, -32}, {-54, -32}}, color = {0, 0, 127}, thickness = 0.5));
        connect(x_base.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-30, -36}, {30, -36}, {30, -56}, {62, -56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(v_est_body.x_b, systemMatricesQ.vel) annotation(
          Line(points = {{-28, 116}, {-100, 116}, {-100, 88}, {-68, 88}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQR.enable) annotation(
          Line(points = {{100, 100}, {50, 100}, {50, 60}}, color = {255, 0, 255}));
        connect(vectorConstant1.v, x_base.v2) annotation(
          Line(points = {{-128, -44}, {-54, -44}, {-54, -40}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.liftoff, accelRef.enable) annotation(
          Line(points = {{100, 100}, {-64, 100}, {-64, -70}}, color = {255, 0, 255}));
        connect(avionicsBus.a_est, accelRef.acc_body) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -100}, {-64, -100}, {-64, -90}}, thickness = 0.5));
        connect(avionicsBus.v_est, accelRef.vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -82}, {-76, -82}}, thickness = 0.5));
        connect(avionicsBus.q_est, accelRef.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -88}, {-76, -88}}, thickness = 0.5));
        connect(track_ref.y, accelRef.track_target) annotation(
          Line(points = {{-142, -72}, {-76, -72}, {-76, -74}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersLQIVnedSimpl;

      model AccGuidance
        outer RocketControl.World.Atmosphere atmosphere;
        extends RocketControl.Components.Interfaces.PartialConditionalEnablePort;
        parameter Real k = 0.7;
        parameter Real kint = 1;
        parameter Real int_lim = 10;
        parameter SI.Angle heading(displayUnit = "deg") = 0;
        parameter SI.Angle flightpathangle(displayUnit = "deg") = from_deg(84);
        final parameter Real V_dir_target_ned[3] = {cos(heading) * cos(flightpathangle), sin(heading) * cos(flightpathangle), -sin(flightpathangle)};
        RocketControl.Components.Interfaces.AvionicsBus bus annotation(
          Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput acc_err_int[2](start = {0, 0}) annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        SI.Velocity V_body[3];
        SI.Velocity V_target_body[3];
        SI.Velocity V_err_body[3];
        SI.Acceleration acc_target[2];
        SI.Acceleration acc_target_sat[2];
        SI.Acceleration acc_meas[2];
        SI.Acceleration acc_err[2];
        SI.Acceleration acc_max;
        SI.Acceleration g[3];
      equation
        g = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, {0, 0, 9.80665});
        V_body = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, bus.v_est);
        V_target_body = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, V_dir_target_ned) * norm(bus.v_est);
        V_err_body = V_target_body - V_body;
        acc_target = k * V_err_body[2:3];
        acc_max = 0.5 * atmosphere.density(-bus.x_est[3]) * norm(bus.v_est) ^ 2 * (3.14 * 0.15 ^ 2 / 4) * 24 * from_deg(4) / 22;
        if noEvent(acc_max < norm(acc_target)) then
          acc_target_sat = acc_target_sat / norm(acc_target) * acc_max;
        else
          acc_target_sat = acc_target;
        end if;
        acc_meas = bus.a_meas[2:3] + g[2:3];
        acc_err = acc_target_sat - acc_meas;
        if noEvent(enable) then
          for i in 1:2 loop
            if abs(acc_err_int[i]) > int_lim and acc_err[i] * acc_err[i] > 0 then
              der(acc_err_int[i]) = 0;
            else
              der(acc_err_int[i]) = kint * acc_err[i];
            end if;
          end for;
        else
          der(acc_err_int) = {0, 0};
        end if;
        annotation(
          Icon(graphics = {Text(origin = {0, 4}, extent = {{-80, 80}, {80, -80}}, textString = "ag")}));
      end AccGuidance;

      model ControllersLQIRates
        RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
          Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput fin[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant Q(n = 9, val = diagonal({0, 0, 0, 1, 0, 0, 100, 1000, 1000})) annotation(
          Placement(visible = true, transformation(origin = {-10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Matrix.MatrixConstant R(n = 3, val = diagonal({100, 100, 100} * 0.1)) annotation(
          Placement(visible = true, transformation(origin = {-10, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.ContinuousLQR continuousLQR(g = -1, m = 3, n = 9, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {40, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate x_base(n1 = 3) annotation(
          Placement(visible = true, transformation(origin = {-68, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
          Placement(visible = true, transformation(origin = {150, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(visible = true, transformation(origin = {-91, 71}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
        RocketControl.World.Atmosphere.Blocks.Density density annotation(
          Placement(visible = true, transformation(origin = {-73, 71}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        RocketControl.Components.Blocks.ned2body ned2body annotation(
          Placement(visible = true, transformation(origin = {-20, 120}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
          Placement(visible = true, transformation(origin = {74, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.FinSaturation finSaturation annotation(
          Placement(visible = true, transformation(origin = {84, 6.66134e-16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        RocketControl.GNC.Control.ErrorState errorState(m = 3, n = 6, p = 3) annotation(
          Placement(visible = true, transformation(origin = {-10, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 6, n2 = 3) annotation(
          Placement(visible = true, transformation(origin = {-10, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Guidance.AngularRateVelocityTrack angularRateGuidance(k = 0.03) annotation(
          Placement(visible = true, transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Guidance.VelocityRef velocityReference(track_ref = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-90, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant roll_rate_ref(k = from_deg(0)) annotation(
          Placement(visible = true, transformation(origin = {-47, -31}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
        RocketControl.GNC.Control.FeedbackIntegrator feedbackIntegrator(default_enabled = false, n = 3, useEnablePort = true) annotation(
          Placement(visible = true, transformation(origin = {10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.SystemMatricesU systemMatricesU(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22) annotation(
          Placement(visible = true, transformation(origin = {-42, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.GNC.Control.LinearLQ.RollAndRatesOutput rollAndRatesOutput annotation(
          Placement(visible = true, transformation(origin = {-64, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(avionicsBus.w_est, x_base.v2) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -14}, {-80, -14}}, thickness = 0.5));
        connect(avionicsBus.x_est[3], gain.u) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 71}, {-95, 71}}, color = {0, 0, 127}));
        connect(gain.y, density.h) annotation(
          Line(points = {{-88, 71}, {-79, 71}}, color = {0, 0, 127}));
        connect(avionicsBus.q_est, ned2body.q_bw) annotation(
          Line(points = {{100, 100}, {16, 100}, {16, 126}, {-8, 126}}, thickness = 0.5));
        connect(avionicsBus.v_est, ned2body.x_w) annotation(
          Line(points = {{100, 100}, {48, 100}, {48, 114}, {-8, 114}}, thickness = 0.5));
        connect(avionicsBus.liftoff, continuousLQR.enable) annotation(
          Line(points = {{100, 100}, {40, 100}, {40, 57}}, color = {255, 0, 255}));
        connect(continuousLQR.u, control2Deflection.u) annotation(
          Line(points = {{51, 48}, {62, 48}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Aaug, continuousLQR.A) annotation(
          Line(points = {{1, 73}, {20, 73}, {20, 56}, {28, 56}}, color = {0, 0, 127}, thickness = 0.5));
        connect(errorState.Baug, continuousLQR.B) annotation(
          Line(points = {{1, 63}, {18, 63}, {18, 52}, {28, 52}}, color = {0, 0, 127}, thickness = 0.5));
        connect(x_base.vc, vectorConcatenate1.v1) annotation(
          Line(points = {{-57, -10}, {-40.5, -10}, {-40.5, -4}, {-22, -4}}, color = {0, 0, 127}, thickness = 0.5));
        connect(control2Deflection.deflection, finSaturation.fin) annotation(
          Line(points = {{86, 48}, {94, 48}, {94, 20}, {72, 20}, {72, 0}, {76, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Q.k, continuousLQR.Q) annotation(
          Line(points = {{1, 40}, {10, 40}, {10, 48}, {28, 48}}, color = {0, 0, 127}, thickness = 0.5));
        connect(R.k, continuousLQR.R) annotation(
          Line(points = {{1, 16}, {12, 16}, {12, 44}, {28, 44}}, color = {0, 0, 127}, thickness = 0.5));
        connect(velocityReference.v_ref, angularRateGuidance.v_ref) annotation(
          Line(points = {{-79, -60}, {-62, -60}}, color = {0, 0, 127}));
        connect(roll_rate_ref.y, feedbackIntegrator.ref[1]) annotation(
          Line(points = {{-39, -31}, {-16, -31}, {-16, -60}, {-2, -60}}, color = {0, 0, 127}));
        connect(angularRateGuidance.w_ref[2], feedbackIntegrator.ref[2]) annotation(
          Line(points = {{-38, -60}, {-2, -60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(angularRateGuidance.w_ref[3], feedbackIntegrator.ref[3]) annotation(
          Line(points = {{-38, -60}, {-2, -60}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus, velocityReference.bus) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -50}}, thickness = 0.5));
        connect(velocityReference.bus, angularRateGuidance.bus) annotation(
          Line(points = {{-100, -50}, {-60, -50}}, thickness = 0.5));
        connect(feedbackIntegrator.err_int, vectorConcatenate1.v2) annotation(
          Line(points = {{21, -60}, {46, -60}, {46, -24}, {-30, -24}, {-30, -12}, {-22, -12}}, color = {0, 0, 127}, thickness = 0.5));
        connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
          Line(points = {{2, -8}, {18, -8}, {18, 40}, {28, 40}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatricesU.A, errorState.A) annotation(
          Line(points = {{-30, 86}, {-28, 86}, {-28, 76}, {-22, 76}}, color = {0, 0, 127}, thickness = 0.5));
        connect(systemMatricesU.B, errorState.B) annotation(
          Line(points = {{-30, 76}, {-28, 76}, {-28, 72}, {-22, 72}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est, systemMatricesU.ang_vel) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 80}, {-54, 80}}, thickness = 0.5));
        connect(density.rho, systemMatricesU.rho) annotation(
          Line(points = {{-68, 72}, {-62, 72}, {-62, 74}, {-54, 74}}, color = {0, 0, 127}));
        connect(continuousLQR.enable, feedbackIntegrator.enable) annotation(
          Line(points = {{40, 58}, {40, -20}, {10, -20}, {10, -51}}, color = {255, 0, 255}));
        connect(ned2body.x_b, x_base.v1) annotation(
          Line(points = {{-30, 120}, {-100, 120}, {-100, -6}, {-80, -6}}, color = {0, 0, 127}, thickness = 0.5));
        connect(rollAndRatesOutput.C, errorState.C) annotation(
          Line(points = {{-52, 46}, {-28, 46}, {-28, 66}, {-22, 66}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.q_est, rollAndRatesOutput.q) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, 46}, {-76, 46}}, thickness = 0.5));
        connect(ned2body.x_b, systemMatricesU.vel) annotation(
          Line(points = {{-30, 120}, {-100, 120}, {-100, 86}, {-54, 86}}, color = {0, 0, 127}, thickness = 0.5));
        connect(avionicsBus.w_est, feedbackIntegrator.feedback) annotation(
          Line(points = {{100, 100}, {-100, 100}, {-100, -102}, {10, -102}, {10, -72}}, thickness = 0.5));
        connect(vectorConstant.v, fin) annotation(
          Line(points = {{162, 70}, {110, 70}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
      protected
        annotation(
          Icon(graphics = {Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
      end ControllersLQIRates;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end OLD;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Lynx;
  package Internal
    partial model PartialRocket
      extends PartialRocketBody;
    equation

      annotation(
        Icon(graphics = {Line(origin = {46, 47}, points = {{54, 23}, {-46, -47}}, color = {0, 255, 0}), Line(origin = {-1, -7}, points = {{-11, 61}, {15, 61}, {1, 61}, {1, -61}, {-15, -61}, {1, -61}, {1, -23}, {15, -23}, {1, -23}, {1, 7}, {-13, 7}}, color = {85, 255, 0}), Text(origin = {-10, -254}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
    end PartialRocket;

    partial model PartialRocketBody
    extends Icon;
    extends Interfaces.PartialLaunchMount;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a ref_center annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    equation

      annotation(
        Icon(graphics = {Line(origin = {-57, -40}, points = {{-37, -20}, {17, -20}, {17, 20}, {37, 20}}), Line(origin = {-57, 41}, points = {{-37, 19}, {17, 19}, {17, -19}, {37, -19}}), Line(origin = {58, 0}, points = {{38, 0}, {-38, 0}}), Text(origin = {469.111, 26}, lineColor = {128, 128, 128}, extent = {{-423.111, -41}, {-311.111, -82}}, textString = "ref_center")}));
    end PartialRocketBody;

    model Icon
    equation

      annotation(
        Icon(coordinateSystem(grid = {2, 0}), graphics = {Polygon(lineColor = {60, 60, 61}, fillColor = {97, 183, 229}, fillPattern = FillPattern.VerticalCylinder, points = {{-20, 60}, {0, 100}, {20, 60}, {20, -60}, {40, -100}, {-40, -100}, {-20, -60}, {-20, 60}})}));
    end Icon;

    partial model PartialSensorsPackage
  extends Modelica.Icons.SensorsPackage;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    equation

      annotation(
        Icon(graphics = {Text(origin = {-10, -254}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
    end PartialSensorsPackage;

    partial block PartialNavigationSystem
    extends RocketControl.GNC.Internal.Icons.Navigation;
    equation

      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end PartialNavigationSystem;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Internal;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Rockets;
