within RocketControl;

package Aerodynamics
  package WithoutControl
    model AerodynamicForce
      extends RocketControl.Aerodynamics.Internal.PartialAerodynamicForce;
      
      class AeroData = RocketControl.Aerodynamics.Internal.AeroData(redeclare type State = State);
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
      extends RocketControl.Aerodynamics.Internal.PartialAerodynamics(redeclare AerodynamicForce aerodynamicForce);
    equation

      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Aerodynamics;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end WithoutControl;

  package Internal
    type Coefficients = enumeration(CN, CM, CA, CY, CLN, CLL, CNA, CMA, CYB, CLNB, CLLB, CNQ, CMQ, CAQ, CNAD, CMAD, CYR, CLNR, CLLR, CYP, CLNP, CLLP);

    function coefficientStrings
      output String array[Coefficients];
    algorithm
      for c in Coefficients loop
        array[c] := String(c);
      end for;
    end coefficientStrings;

    package AerodynamicsWithControl
      model AerodynamicForce
        import Modelica.Mechanics.MultiBody.Frames;
        import Modelica.Math.Vectors.norm;
        type AeroCoeffictient = Real(unit = "1");
        outer World.Atmosphere atmosphere;
        AeroData aerodata = AeroData();
        parameter Modelica.Units.SI.Angle max_alpha = Modelica.Units.Conversions.from_deg(75);
        parameter Modelica.Units.SI.Angle min_alpha = Modelica.Units.Conversions.from_deg(-75);
        parameter Modelica.Units.SI.Angle max_beta = Modelica.Units.Conversions.from_deg(75);
        parameter Modelica.Units.SI.Angle min_beta = Modelica.Units.Conversions.from_deg(-75);
        parameter Modelica.Units.SI.Length d = 0.15;
        parameter Modelica.Units.SI.Area S = Modelica.Constants.pi * (0.15 / 2) ^ 2;
        AeroCoeffictient coeffs[Coeff];
        Real state_arr[State];
        SI.Angle alpha0;
        SI.Angle beta0;
        AeroCoeffictient CA;
        AeroCoeffictient CY;
        AeroCoeffictient CN;
        AeroCoeffictient CLL;
        AeroCoeffictient CLM;
        AeroCoeffictient CLN;
        SI.Velocity v_norm;
        SI.Force fa[3];
        SI.Torque ma[3];
        Modelica.Units.SI.Pressure q(displayUnit = "Pa");
        Real q_v(unit = "kg/(m2.s)");
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
        RocketControl.Interfaces.AeroStateInput aeroState annotation(
          Placement(visible = true, transformation(origin = {-100, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-110, 60}, {-90, 80}}, rotation = 0)));
        RocketControl.Interfaces.FinDeflectionInput finDeflectionInput[4] annotation(
          Placement(visible = true, transformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
//  assert(aeroState.alpha <= max_alpha and aeroState.alpha >= min_alpha, "Angle of attack out of range");
//  assert(aeroState.beta <= max_beta and aeroState.beta >= min_beta, "Sideslip angle out of range");
        alpha0 = aeroState.alpha;
        beta0 = aeroState.beta;
        v_norm = norm(aeroState.v);
        q_v = 0.5 * atmosphere.density(frame_b.r_0) * v_norm;
        q = q_v * v_norm;
        state_arr[State.alpha] = Modelica.Units.Conversions.to_deg(aeroState.alpha);
        state_arr[State.beta] = Modelica.Units.Conversions.to_deg(aeroState.beta);
        state_arr[State.mach] = aeroState.mach;
        state_arr[State.altitude] = aeroState.altitude;
        state_arr[State.fin2delta1] = finDeflectionInput[1].deflection;
        state_arr[State.fin2delta2] = finDeflectionInput[2].deflection;
        state_arr[State.fin2delta3] = finDeflectionInput[3].deflection;
        state_arr[State.fin2delta4] = finDeflectionInput[4].deflection;
        coeffs = coeffValue(aerodata, state_arr);
        CA = coeffs[C.CA];
        CY = coeffs[C.CY] + coeffs[C.CYB] * (aeroState.beta - beta0);
        CN = coeffs[C.CN] + coeffs[C.CNA] * (aeroState.alpha - alpha0);
        CLL = coeffs[C.CLL] + coeffs[C.CLLB] * (aeroState.beta - beta0);
        CLM = coeffs[C.CM] + coeffs[C.CMA] * (aeroState.alpha - alpha0);
        CLN = coeffs[C.CLN] + coeffs[C.CLNB] * (aeroState.beta - beta0);
        fa[1] = (-q * S * CA) - q_v * S * coeffs[C.CAQ] * aeroState.w[2] * d;
        fa[2] = q * S * CY + q_v * S * (coeffs[C.CYP] * aeroState.w[1] + coeffs[C.CYR] * aeroState.w[3]);
        fa[3] = (-q * S * CN) - q_v * S * (coeffs[C.CNQ] * aeroState.w[2] + coeffs[C.CNAD] * der(aeroState.alpha));
        ma[1] = q_v * S * d * (v_norm * CLL + (coeffs[C.CLLP] * aeroState.w[1] + coeffs[C.CLLR] * aeroState.w[3]) * d / 2);
        ma[2] = q_v * S * d * (v_norm * CLM + (coeffs[C.CMAD] * der(aeroState.alpha) + coeffs[C.CMQ] * aeroState.w[2]) * d / 2);
        ma[3] = q_v * S * d * (v_norm * CLN + (coeffs[C.CLNR] * aeroState.w[3] + coeffs[C.CLNP] * aeroState.w[1]) * d / 2);
        frame_b.f = -fa;
        frame_b.t = -ma;
        annotation(
          Icon(graphics = {Polygon(origin = {4, 6}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, points = {{-58, 24}, {-74, 18}, {-80, 8}, {-76, -4}, {-66, -10}, {-52, -12}, {-36, -12}, {-22, -12}, {8, -12}, {42, -16}, {80, -24}, {54, -8}, {22, 6}, {-4, 16}, {-32, 22}, {-58, 24}}), Line(origin = {-2, 11.9985}, points = {{-88, 6.00147}, {-82, 6.00147}, {-72, 18.0015}, {-52, 24.0015}, {-26, 22.0015}, {6, 16.0015}, {36, 4.00147}, {70, -11.9985}, {88, -23.9985}, {98, -23.9985}}), Line(origin = {-1, 17.9989}, points = {{-89, 6.00111}, {-85, 6.00111}, {-75, 18.0011}, {-55, 24.0011}, {-25, 22.0011}, {5, 16.0011}, {71, -11.9989}, {87, -23.9989}, {99, -23.9989}}), Line(origin = {-0.845794, -7.65421}, points = {{-88.5, 17}, {-80.5, 17}, {-76.5, 7}, {-64.5, -3}, {-38.5, -5}, {15.5, -5}, {49.5, -9}, {83.5, -17}, {99.5, -17}, {97.5, -17}}), Line(origin = {0.327103, -17.5981}, points = {{-90, 20}, {-88, 20}, {-70, 0}, {-42, -2}, {16, -2}, {50, -6}, {82, -14}, {100, -14}}), Polygon(origin = {-17, 51}, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Text(origin = {11, 84}, extent = {{-25, 26}, {25, -26}}, textString = "L"), Text(origin = {69, 42}, extent = {{-25, 26}, {25, -26}}, textString = "D"), Polygon(origin = {21, 17}, rotation = -90, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Text(origin = {2, -178}, lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name")}));
      end AerodynamicForce;

      class AeroData
        extends ExternalObject;

        function constructor
          input String states_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_states.npz");
          input String coeffs_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_coeffs.npz");
          input String state_names[State] = stateStrings();
          input String coeff_names[Aerodynamics.Coeff] = coefficientStrings();
          output AeroData aerodata;
        
          external "C" aerodata = initAeroData(states_file, coeffs_file, state_names, size(state_names, 1), coeff_names, size(coeff_names, 1)) annotation(
            Library = {"aerodata", "cnpy", "z"},
            Include = "#include \"ModelicaAeroData.h\"");
        end constructor;

        function destructor
          input AeroData aerodata;
        
          external "C" closeAeroData(aerodata) annotation(
            Library = {"aerodata", "cnpy"},
            Include = "#include \"ModelicaAeroData.h\"");
        end destructor;
      end AeroData;

      function stateStrings
        output String array[State];
      algorithm
        for s in State loop
          array[s] := String(s);
        end for;
      end stateStrings;

      function coeffValue
        input AeroData aerodata;
        input Real[State] state;
        output Real[Coeff] coefficients;
      
        external getAeroCoefficients(aerodata, state, size(state, 1), coefficients, size(coefficients, 1)) annotation(
          Library = {"aerodata", "cnpy", "z"},
          Include = "#include \"ModelicaAeroData.h\"");
      end coeffValue;

      model Aerodynamics
        RocketControl.Aerodynamics.AerodynamicForce aerodynamicForce annotation(
          Placement(visible = true, transformation(origin = {88, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        RocketControl.Components.Sensors.AeroStateSensor aeroStateSensor annotation(
          Placement(visible = true, transformation(origin = {-36, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
        RocketControl.Interfaces.FinDeflectionInput finDeflectionInput[4] annotation(
          Placement(visible = true, transformation(origin = {-102, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(aeroStateSensor.aeroStateOutput, aerodynamicForce.aeroState) annotation(
          Line(points = {{-26, -4}, {-3, -4}, {-3, -29}, {78, -29}}));
        connect(frame_b, aeroStateSensor.frame_a) annotation(
          Line(points = {{-100, 0}, {-73, 0}, {-73, -4}, {-46, -4}}));
        connect(aerodynamicForce.frame_b, frame_b) annotation(
          Line(points = {{98, -36}, {-80, -36}, {-80, 0}, {-100, 0}}));
        connect(finDeflectionInput, aerodynamicForce.finDeflectionInput) annotation(
          Line(points = {{-102, -70}, {28, -70}, {28, -40}, {78, -40}}, thickness = 0.5));
        annotation(
          Icon(graphics = {Line(origin = {-1, 17.9989}, points = {{-89, 6.00111}, {-85, 6.00111}, {-75, 18.0011}, {-55, 24.0011}, {-25, 22.0011}, {5, 16.0011}, {71, -11.9989}, {87, -23.9989}, {99, -23.9989}}), Line(origin = {0.327103, -17.5981}, points = {{-90, 20}, {-88, 20}, {-70, 0}, {-42, -2}, {16, -2}, {50, -6}, {82, -14}, {100, -14}}), Polygon(origin = {4, 6}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, points = {{-58, 24}, {-74, 18}, {-80, 8}, {-76, -4}, {-66, -10}, {-52, -12}, {-36, -12}, {-22, -12}, {8, -12}, {42, -16}, {80, -24}, {54, -8}, {22, 6}, {-4, 16}, {-32, 22}, {-58, 24}}), Text(origin = {11, 84}, extent = {{-25, 26}, {25, -26}}, textString = "L"), Text(origin = {69, 42}, extent = {{-25, 26}, {25, -26}}, textString = "D"), Line(origin = {-0.845794, -7.65421}, points = {{-88.5, 17}, {-80.5, 17}, {-76.5, 7}, {-64.5, -3}, {-38.5, -5}, {15.5, -5}, {49.5, -9}, {83.5, -17}, {99.5, -17}, {97.5, -17}}), Line(origin = {-2, 11.9985}, points = {{-88, 6.00147}, {-82, 6.00147}, {-72, 18.0015}, {-52, 24.0015}, {-26, 22.0015}, {6, 16.0015}, {36, 4.00147}, {70, -11.9985}, {88, -23.9985}, {98, -23.9985}}), Polygon(origin = {25, 17}, rotation = -90, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Polygon(origin = {-17, 51}, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Text(origin = {10, -170}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
      end Aerodynamics;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AerodynamicsWithControl;

    class AeroData
      type E = enumeration(:);
      replaceable type State = E;

      class ExternalAeroData
        extends ExternalObject;

        function constructor
          input String states_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_states.npz");
          input String coeffs_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_coeffs.npz");
          input String state_names[State] = stateStrings();
          input String coeff_names[Internal.Coefficients] = Internal.coefficientStrings();
          output ExternalAeroData aerodata;
        
          external "C" aerodata = initAeroData(states_file, coeffs_file, state_names, size(state_names, 1), coeff_names, size(coeff_names, 1)) annotation(
            Library = {"aerodata", "cnpy", "z"},
            Include = "#include \"ModelicaAeroData.h\"");
        end constructor;

        function destructor
          input ExternalAeroData aerodata;
        
          external "C" closeAeroData(aerodata) annotation(
            Library = {"aerodata", "cnpy"},
            Include = "#include \"ModelicaAeroData.h\"");
        end destructor;
      end ExternalAeroData;

      function stateStrings
        output String array[State];
      algorithm
        for s in State loop
          array[s] := String(s);
        end for;
      end stateStrings;

      function getData
        input ExternalAeroData aerodata;
        input Real[State] state;
        output Real[RocketControl.Aerodynamics.Internal.Coefficients] coefficients;
      
        external getAeroCoefficients(aerodata, state, size(state, 1), coefficients, size(coefficients, 1)) annotation(
          Library = {"aerodata", "cnpy", "z"},
          Include = "#include \"ModelicaAeroData.h\"");
      end getData;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AeroData;

    partial model PartialAerodynamicForce
      import Modelica.Mechanics.MultiBody.Frames;
      import Modelica.Math.Vectors.norm;
      type AeroCoeffictient = Real(unit = "1");
      type C = Coefficients;
      outer World.Atmosphere atmosphere;
      parameter Modelica.Units.SI.Angle max_alpha = Modelica.Units.Conversions.from_deg(75);
      parameter Modelica.Units.SI.Angle min_alpha = Modelica.Units.Conversions.from_deg(-75);
      parameter Modelica.Units.SI.Angle max_beta = Modelica.Units.Conversions.from_deg(75);
      parameter Modelica.Units.SI.Angle min_beta = Modelica.Units.Conversions.from_deg(-75);
      parameter Modelica.Units.SI.Length d = 0.15;
      parameter Modelica.Units.SI.Area S = Modelica.Constants.pi * (0.15 / 2) ^ 2;
      AeroCoeffictient coeffs[Coefficients];
      SI.Angle alpha0;
      SI.Angle beta0;
      AeroCoeffictient CA;
      AeroCoeffictient CY;
      AeroCoeffictient CN;
      AeroCoeffictient CLL;
      AeroCoeffictient CLM;
      AeroCoeffictient CLN;
      SI.Velocity v_norm;
      SI.Force fa[3];
      SI.Torque ma[3];
      Modelica.Units.SI.Pressure q(displayUnit = "Pa");
      Real q_v(unit = "kg/(m2.s)");
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      RocketControl.Interfaces.AeroStateInput aeroState annotation(
        Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-110, -10}, {-90, 10}}, rotation = 0)));
    equation
//  assert(aeroState.alpha <= max_alpha and aeroState.alpha >= min_alpha, "Angle of attack out of range");
//  assert(aeroState.beta <= max_beta and aeroState.beta >= min_beta, "Sideslip angle out of range");
      alpha0 = aeroState.alpha;
      beta0 = aeroState.beta;
      v_norm = norm(aeroState.v);
      q_v = 0.5 * atmosphere.density(frame_b.r_0) * v_norm;
      q = q_v * v_norm;
      CA = coeffs[C.CA];
      CY = coeffs[C.CY] + coeffs[C.CYB] * (aeroState.beta - beta0);
      CN = coeffs[C.CN] + coeffs[C.CNA] * (aeroState.alpha - alpha0);
      CLL = coeffs[C.CLL] + coeffs[C.CLLB] * (aeroState.beta - beta0);
      CLM = coeffs[C.CM] + coeffs[C.CMA] * (aeroState.alpha - alpha0);
      CLN = coeffs[C.CLN] + coeffs[C.CLNB] * (aeroState.beta - beta0);
      fa[1] = (-q * S * CA) - q_v * S * coeffs[C.CAQ] * aeroState.w[2] * d;
      fa[2] = q * S * CY + q_v * S * (coeffs[C.CYP] * aeroState.w[1] + coeffs[C.CYR] * aeroState.w[3]);
      fa[3] = (-q * S * CN) - q_v * S * (coeffs[C.CNQ] * aeroState.w[2] + coeffs[C.CNAD] * der(aeroState.alpha));
      ma[1] = q_v * S * d * (v_norm * CLL + (coeffs[C.CLLP] * aeroState.w[1] + coeffs[C.CLLR] * aeroState.w[3]) * d / 2);
      ma[2] = q_v * S * d * (v_norm * CLM + (coeffs[C.CMAD] * der(aeroState.alpha) + coeffs[C.CMQ] * aeroState.w[2]) * d / 2);
      ma[3] = q_v * S * d * (v_norm * CLN + (coeffs[C.CLNR] * aeroState.w[3] + coeffs[C.CLNP] * aeroState.w[1]) * d / 2);
      frame_b.f = -fa;
      frame_b.t = -ma;
      annotation(
        Icon(graphics = {Polygon(origin = {4, 6}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, points = {{-58, 24}, {-74, 18}, {-80, 8}, {-76, -4}, {-66, -10}, {-52, -12}, {-36, -12}, {-22, -12}, {8, -12}, {42, -16}, {80, -24}, {54, -8}, {22, 6}, {-4, 16}, {-32, 22}, {-58, 24}}), Line(origin = {-2, 11.9985}, points = {{-88, 6.00147}, {-82, 6.00147}, {-72, 18.0015}, {-52, 24.0015}, {-26, 22.0015}, {6, 16.0015}, {36, 4.00147}, {70, -11.9985}, {88, -23.9985}, {98, -23.9985}}), Line(origin = {-1, 17.9989}, points = {{-89, 6.00111}, {-85, 6.00111}, {-75, 18.0011}, {-55, 24.0011}, {-25, 22.0011}, {5, 16.0011}, {71, -11.9989}, {87, -23.9989}, {99, -23.9989}}), Line(origin = {-0.845794, -7.65421}, points = {{-88.5, 17}, {-80.5, 17}, {-76.5, 7}, {-64.5, -3}, {-38.5, -5}, {15.5, -5}, {49.5, -9}, {83.5, -17}, {99.5, -17}, {97.5, -17}}), Line(origin = {0.327103, -17.5981}, points = {{-90, 20}, {-88, 20}, {-70, 0}, {-42, -2}, {16, -2}, {50, -6}, {82, -14}, {100, -14}}), Polygon(origin = {-17, 51}, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Text(origin = {11, 84}, extent = {{-25, 26}, {25, -26}}, textString = "L"), Text(origin = {69, 42}, extent = {{-25, 26}, {25, -26}}, textString = "D"), Polygon(origin = {21, 17}, rotation = -90, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Text(origin = {2, -178}, lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name")}));
    end PartialAerodynamicForce;

    model PartialAerodynamics
      RocketControl.Components.Sensors.AeroStateSensor aeroStateSensor annotation(
        Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      replaceable RocketControl.Aerodynamics.Internal.PartialAerodynamicForce aerodynamicForce annotation(
        Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(frame_b, aeroStateSensor.frame_a) annotation(
        Line(points = {{-100, 0}, {-40, 0}}));
      connect(aeroStateSensor.aeroStateOutput, aerodynamicForce.aeroState) annotation(
        Line(points = {{-20, 0}, {40, 0}}));
      connect(aerodynamicForce.frame_b, frame_b) annotation(
        Line(points = {{60, 0}, {80, 0}, {80, -40}, {-74, -40}, {-74, 0}, {-100, 0}}));
      annotation(
        Icon(graphics = {Line(origin = {-1, 17.9989}, points = {{-89, 6.00111}, {-85, 6.00111}, {-75, 18.0011}, {-55, 24.0011}, {-25, 22.0011}, {5, 16.0011}, {71, -11.9989}, {87, -23.9989}, {99, -23.9989}}), Line(origin = {0.327103, -17.5981}, points = {{-90, 20}, {-88, 20}, {-70, 0}, {-42, -2}, {16, -2}, {50, -6}, {82, -14}, {100, -14}}), Polygon(origin = {4, 6}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, points = {{-58, 24}, {-74, 18}, {-80, 8}, {-76, -4}, {-66, -10}, {-52, -12}, {-36, -12}, {-22, -12}, {8, -12}, {42, -16}, {80, -24}, {54, -8}, {22, 6}, {-4, 16}, {-32, 22}, {-58, 24}}), Text(origin = {11, 84}, extent = {{-25, 26}, {25, -26}}, textString = "L"), Text(origin = {69, 42}, extent = {{-25, 26}, {25, -26}}, textString = "D"), Line(origin = {-0.845794, -7.65421}, points = {{-88.5, 17}, {-80.5, 17}, {-76.5, 7}, {-64.5, -3}, {-38.5, -5}, {15.5, -5}, {49.5, -9}, {83.5, -17}, {99.5, -17}, {97.5, -17}}), Line(origin = {-2, 11.9985}, points = {{-88, 6.00147}, {-82, 6.00147}, {-72, 18.0015}, {-52, 24.0015}, {-26, 22.0015}, {6, 16.0015}, {36, 4.00147}, {70, -11.9985}, {88, -23.9985}, {98, -23.9985}}), Polygon(origin = {25, 17}, rotation = -90, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Polygon(origin = {-17, 51}, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Text(origin = {10, -170}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
    end PartialAerodynamics;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Internal;
  
  package WithControl
    model AerodynamicForce
      extends RocketControl.Aerodynamics.Internal.PartialAerodynamicForce;
      
      class AeroData = RocketControl.Aerodynamics.Internal.AeroData(redeclare type State = State);
      
      AeroData.ExternalAeroData aerodata = AeroData.ExternalAeroData(states_file, coeffs_file);
      
      Real state_arr[State];
  RocketControl.Interfaces.FinDeflectionInput finDeflectionInput[4] annotation(
        Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
      constant String states_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_states_fins.npz");
      constant String coeffs_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_coeffs_fins.npz");
    equation
    
      state_arr[State.alpha] = Modelica.Units.Conversions.to_deg(aeroState.alpha);
      state_arr[State.beta] = Modelica.Units.Conversions.to_deg(aeroState.beta);
      state_arr[State.mach] = aeroState.mach;
      state_arr[State.altitude] = aeroState.altitude;
      state_arr[State.fin2delta1] = finDeflectionInput[1].deflection;
      state_arr[State.fin2delta2] = finDeflectionInput[2].deflection;
      state_arr[State.fin2delta3] = finDeflectionInput[3].deflection;
      state_arr[State.fin2delta4] = finDeflectionInput[4].deflection;
      
      coeffs = AeroData.getData(aerodata, state_arr);
     
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AerodynamicForce;
  
    type State = enumeration(alpha, mach, beta, altitude, fin2delta1, fin2delta2, fin2delta3, fin2delta4);
  
    model Aerodynamics
      extends RocketControl.Aerodynamics.Internal.PartialAerodynamics(redeclare AerodynamicForce aerodynamicForce);
  RocketControl.Interfaces.FinDeflectionInput finDeflectionInput[4] annotation(
        Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
    connect(finDeflectionInput, aerodynamicForce.finDeflectionInput)
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Aerodynamics;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end WithControl;
end Aerodynamics;
