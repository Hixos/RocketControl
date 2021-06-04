within RocketControl;

package Aerodynamics
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
    Modelica.Units.SI.Pressure q(displayUnit="Pa");
    Real q_v(unit="kg/(m2.s)");
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  RocketControl.Interfaces.AeroStateInput aeroState annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-112, -10}, {-92, 10}}, rotation = 0)));
  equation
  //  assert(aeroState.alpha <= max_alpha and aeroState.alpha >= min_alpha, "Angle of attack out of range");
  //  assert(aeroState.beta <= max_beta and aeroState.beta >= min_beta, "Sideslip angle out of range");
    
    alpha0 = aeroState.alpha;
    beta0 = aeroState.beta;
   
    
    v_norm = norm(aeroState.v);
    q_v = 0.5 * atmosphere.density(frame_b.r_0) * v_norm;
    q = q_v * v_norm;
    
    
    state_arr[State.alphas] = Modelica.Units.Conversions.to_deg(aeroState.alpha);
    state_arr[State.betas] = Modelica.Units.Conversions.to_deg(aeroState.beta);
    state_arr[State.machs] = aeroState.mach;
    state_arr[State.alts] = aeroState.altitude;
    coeffs = coeffValue(aerodata, state_arr);
    
    CA = coeffs[Coeff.CA];
    CY = coeffs[Coeff.CY] + coeffs[Coeff.CYB] * (aeroState.beta - beta0);
    CN = coeffs[Coeff.CN] + coeffs[Coeff.CNA] * (aeroState.alpha - alpha0);
    
    CLL = coeffs[Coeff.CLL] + coeffs[Coeff.CLLB] * (aeroState.beta - beta0);
    CLM = coeffs[Coeff.CM] + coeffs[Coeff.CMA] * (aeroState.alpha - alpha0);
    CLN = coeffs[Coeff.CLN] + coeffs[Coeff.CLNB] * (aeroState.beta - beta0);
      
    fa[1] = - q * S * CA - q_v * S * coeffs[Coeff.CAQ] * aeroState.w[2] * d;
    fa[2] = q * S * CY + q_v * S * (coeffs[Coeff.CYP] * aeroState.w[1] + coeffs[Coeff.CYR] * aeroState.w[3]);
    fa[3] = - q * S * CN - q_v * S * (coeffs[Coeff.CNQ] * aeroState.w[2] + coeffs[Coeff.CNAD]  * der(aeroState.alpha));
    
    ma[1] = q_v * S * d * (v_norm * CLL + (coeffs[Coeff.CLLP] * aeroState.w[1] + coeffs[Coeff.CLLR] * aeroState.w[3]) * d / 2);
    ma[2] = q_v * S * d * (v_norm * CLM + (coeffs[Coeff.CMAD] * der(aeroState.alpha) + coeffs[Coeff.CMQ] * aeroState.w[2]) * d / 2);
    ma[3] = q_v * S * d * (v_norm * CLN + (coeffs[Coeff.CLNR] * aeroState.w[3] + coeffs[Coeff.CLNP] * aeroState.w[1]) * d / 2);
    
    frame_b.f = -fa;
    frame_b.t = -ma;
  
    annotation(
      Icon(graphics = {Polygon(origin = {4, 6}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, points = {{-58, 24}, {-74, 18}, {-80, 8}, {-76, -4}, {-66, -10}, {-52, -12}, {-36, -12}, {-22, -12}, {8, -12}, {42, -16}, {80, -24}, {54, -8}, {22, 6}, {-4, 16}, {-32, 22}, {-58, 24}}), Line(origin = {-2, 11.9985}, points = {{-88, 6.00147}, {-82, 6.00147}, {-72, 18.0015}, {-52, 24.0015}, {-26, 22.0015}, {6, 16.0015}, {36, 4.00147}, {70, -11.9985}, {88, -23.9985}, {98, -23.9985}}), Line(origin = {-1, 17.9989}, points = {{-89, 6.00111}, {-85, 6.00111}, {-75, 18.0011}, {-55, 24.0011}, {-25, 22.0011}, {5, 16.0011}, {71, -11.9989}, {87, -23.9989}, {99, -23.9989}}), Line(origin = {-0.845794, -7.65421}, points = {{-88.5, 17}, {-80.5, 17}, {-76.5, 7}, {-64.5, -3}, {-38.5, -5}, {15.5, -5}, {49.5, -9}, {83.5, -17}, {99.5, -17}, {97.5, -17}}), Line(origin = {0.327103, -17.5981}, points = {{-90, 20}, {-88, 20}, {-70, 0}, {-42, -2}, {16, -2}, {50, -6}, {82, -14}, {100, -14}}), Polygon(origin = {-17, 51}, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Text(origin = {11, 84}, extent = {{-25, 26}, {25, -26}}, textString = "L"), Text(origin = {69, 42}, extent = {{-25, 26}, {25, -26}}, textString = "D"), Polygon(origin = {21, 17}, rotation = -90, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Text(origin = {2, -178},lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name")}));
  end AerodynamicForce;

  type Coeff = enumeration(CN, CM, CA, CY, CLN, CLL, CNA, CMA, CYB, CLNB, CLLB, CNQ, CMQ, CAQ, CNAD, CMAD, CYR, CLNR, CLLR, CYP, CLNP, CLLP);

  function coefficientStrings
    output String array[Coeff];
  algorithm
    for c in Coeff loop
      array[c] := String(c);
    end for;
  end coefficientStrings;

  class AeroData
    extends ExternalObject;

    function constructor
      input String states_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_states.npz");
      input String coeffs_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_coeffs.npz");
      input String state_names[State] = stateStrings();
      input String coeff_names[Coeff] = coefficientStrings();
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

  type State = enumeration(alphas, machs, betas, alts);

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
      Library = {"aerodata", "cnpy"},
      Include = "#include \"ModelicaAeroData.h\"");
  end coeffValue;

  model Aerodynamics
  RocketControl.Aerodynamics.AerodynamicForce aerodynamicForce annotation(
      Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.AeroStateSensor aeroStateSensor annotation(
      Placement(visible = true, transformation(origin = {-36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  equation
    connect(aeroStateSensor.aeroStateOutput, aerodynamicForce.aeroState) annotation(
      Line(points = {{-26, 0}, {20, 0}}));
  connect(frame_b, aeroStateSensor.frame_a) annotation(
      Line(points = {{-100, 0}, {-46, 0}}));
  connect(aerodynamicForce.frame_b, frame_b) annotation(
      Line(points = {{40, 0}, {60, 0}, {60, -40}, {-80, -40}, {-80, 0}, {-100, 0}}));
  annotation(
      Icon(graphics = {Line(origin = {-1, 17.9989}, points = {{-89, 6.00111}, {-85, 6.00111}, {-75, 18.0011}, {-55, 24.0011}, {-25, 22.0011}, {5, 16.0011}, {71, -11.9989}, {87, -23.9989}, {99, -23.9989}}), Line(origin = {0.327103, -17.5981}, points = {{-90, 20}, {-88, 20}, {-70, 0}, {-42, -2}, {16, -2}, {50, -6}, {82, -14}, {100, -14}}), Polygon(origin = {4, 6}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, points = {{-58, 24}, {-74, 18}, {-80, 8}, {-76, -4}, {-66, -10}, {-52, -12}, {-36, -12}, {-22, -12}, {8, -12}, {42, -16}, {80, -24}, {54, -8}, {22, 6}, {-4, 16}, {-32, 22}, {-58, 24}}), Text(origin = {11, 84}, extent = {{-25, 26}, {25, -26}}, textString = "L"), Text(origin = {69, 42}, extent = {{-25, 26}, {25, -26}}, textString = "D"), Line(origin = {-0.845794, -7.65421}, points = {{-88.5, 17}, {-80.5, 17}, {-76.5, 7}, {-64.5, -3}, {-38.5, -5}, {15.5, -5}, {49.5, -9}, {83.5, -17}, {99.5, -17}, {97.5, -17}}), Line(origin = {-2, 11.9985}, points = {{-88, 6.00147}, {-82, 6.00147}, {-72, 18.0015}, {-52, 24.0015}, {-26, 22.0015}, {6, 16.0015}, {36, 4.00147}, {70, -11.9985}, {88, -23.9985}, {98, -23.9985}}), Polygon(origin = {25, 17}, rotation = -90, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Polygon(origin = {-17, 51}, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Text(origin = {10, -170}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));end Aerodynamics;
end Aerodynamics;
