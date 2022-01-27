within RocketControl;

package Aerodynamics
  extends Internal.Icons.AerodynamicsIcon;

partial model PartialAerodynamicForce
extends Internal.Icons.AerodynamicsIcon;
  import Modelica.Mechanics.MultiBody.Frames;
  type AeroCoefficient = Real(unit = "1");
  type C = Coefficients;
  outer World.Atmosphere atmosphere;
  outer World.MyWorld world;
  parameter Modelica.Units.SI.Angle max_alpha = from_deg(10);
  parameter Modelica.Units.SI.Angle min_alpha = from_deg(-10);
  parameter Modelica.Units.SI.Angle max_beta = from_deg(10);
  parameter Modelica.Units.SI.Angle min_beta = from_deg(-10);
  parameter Modelica.Units.SI.Length d = 0.15;
  parameter Modelica.Units.SI.Area S = pi * (0.15 / 2) ^ 2;
  
  AeroCoefficient coeffs[Coefficients];
  SI.Angle alpha0;
  SI.Angle beta0;
  AeroCoefficient CA;
  
  AeroCoefficient CY;
  AeroCoefficient CN;
  AeroCoefficient CLL;
  AeroCoefficient CLM;
  AeroCoefficient CLN;
  SI.Velocity v_norm;
  SI.Force fa[3];
  SI.Torque ma[3];
  Modelica.Units.SI.Pressure q(displayUnit = "Pa");
  Real q_v(unit = "kg/(m2.s)");
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Aerodynamics.Interfaces.AeroStateInput aeroState annotation(
    Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-110, -10}, {-90, 10}}, rotation = 0)));
equation
//  assert(aeroState.alpha <= max_alpha and aeroState.alpha >= min_alpha, "Angle of attack out of range");
//  assert(aeroState.beta <= max_beta and aeroState.beta >= min_beta, "Sideslip angle out of range");
// TODO: alpha0 / beta0 should be the nearest grid points in the case of nearest neighbour interpolation
  alpha0 = max(min(aeroState.alpha,max_alpha), min_alpha);
  beta0 = max(min(aeroState.beta,max_beta), min_beta);
  
  v_norm = norm(aeroState.v);
  q_v = 0.5 * atmosphere.density(world.altitude(frame_b.r_0)) * v_norm;
  q = q_v * v_norm;
  
  CA = coeffs[C.CA];
  CY = coeffs[C.CY] + coeffs[C.CYB] * (aeroState.beta - beta0);
// Second term is always zero in case of linear interpoaltion of the coefficients
  CN = coeffs[C.CN] + coeffs[C.CNA] * (aeroState.alpha - alpha0);
  CLL = coeffs[C.CLL] + coeffs[C.CLLB] * (aeroState.beta - beta0);
  CLM = coeffs[C.CM] + coeffs[C.CMA] * (aeroState.alpha - alpha0);
  CLN = coeffs[C.CLN] + coeffs[C.CLNB] * (aeroState.beta - beta0);
  fa[1] = (-q * S * CA) - q_v * S * coeffs[C.CAQ] * aeroState.w[2] * d;
  fa[2] = q * S * CY + q_v * S * (coeffs[C.CYP] * aeroState.w[1] + coeffs[C.CYR] * aeroState.w[3])*d;
  fa[3] = (-q * S * CN) - q_v * S * (coeffs[C.CNQ] * aeroState.w[2] + coeffs[C.CNAD] * aeroState.alpha_dot)*d;
  ma[1] = q_v * S * d * (v_norm * CLL + (coeffs[C.CLLP] * aeroState.w[1] + coeffs[C.CLLR] * aeroState.w[3]) * d / 2);
  ma[2] = q_v * S * d * (v_norm * CLM + (coeffs[C.CMAD] * der(aeroState.alpha) + coeffs[C.CMQ] * aeroState.w[2]) * d / 2);
  ma[3] = q_v * S * d * (v_norm * CLN + (coeffs[C.CLNR] * aeroState.w[3] + coeffs[C.CLNP] * aeroState.w[1]) * d / 2);
  frame_b.f = -fa;
  frame_b.t = -ma;
  annotation(
    Icon(graphics = {Text(origin = {2, -178}, lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name")}));
end PartialAerodynamicForce;

  model PartialAerodynamics
  extends Internal.Icons.AerodynamicsIcon;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    replaceable PartialAerodynamicForce aerodynamicForce annotation(
      Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.AeroStateSensor aeroStateSensor annotation(
      Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(aerodynamicForce.frame_b, frame_b) annotation(
      Line(points = {{60, 0}, {80, 0}, {80, -40}, {-74, -40}, {-74, 0}, {-100, 0}}));
  connect(frame_b, aeroStateSensor.frame_a) annotation(
      Line(points = {{-100, 0}, {-40, 0}}));
  connect(aeroStateSensor.aeroStateOutput, aerodynamicForce.aeroState) annotation(
      Line(points = {{-20, 0}, {40, 0}}));
    annotation(
      Icon(graphics = {Text(origin = {10, -170}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
  end PartialAerodynamics;

  model AeroStateSensor
    extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
    import Modelica.Mechanics.MultiBody.Frames;
    import Modelica.Math.Vectors;
    RocketControl.Aerodynamics.AeroAnglesSensor aeroAnglesSensor annotation(
      Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    outer World.Atmosphere atmosphere;
    outer World.MyWorld world;
    Modelica.Units.SI.Angle beta2;
    SI.Velocity[3] v_w;
    SI.Velocity[3] v;
    Aerodynamics.Interfaces.AeroStateOutput aeroStateOutput annotation(
      Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
      Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    v_w = atmosphere.windSpeed(world.altitude(frame_a.r_0));
    aeroStateOutput.alpha = aeroAnglesSensor.alpha;
    aeroStateOutput.alpha_dot = aeroAnglesSensor.alpha_dot;
    aeroStateOutput.beta = aeroAnglesSensor.beta;
    aeroStateOutput.mach = norm(v) / atmosphere.speedOfSound(world.altitude(frame_a.r_0));
    aeroStateOutput.altitude = world.altitude(frame_a.r_0);
    v = Frames.resolve2(frame_a.R, der(frame_a.r_0) - v_w);
    aeroStateOutput.v = v;
    aeroStateOutput.w = absoluteAngularVelocity.w;
    beta2 = atan2(v[2], v[1]);
    connect(aeroAnglesSensor.frame_a, frame_a) annotation(
      Line(points = {{-10, 40}, {-55, 40}, {-55, 0}, {-100, 0}}));
    connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
      Line(points = {{-100, 0}, {-54, 0}, {-54, -40}, {-10, -40}}));
    annotation(
      Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Text(origin = {1, -40}, lineColor = {64, 64, 64}, extent = {{-101, 20}, {101, -20}}, textString = "aero"), Line(origin = {84.1708, -2.17082}, points = {{-14.1708, 2.17082}, {11.8292, 2.17082}, {13.8292, -1.82918}})}));
  end AeroStateSensor;

  model AeroAnglesSensor
    extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
    import Modelica.Mechanics.MultiBody.Frames;
    import Modelica.Math.Vectors;
    outer World.Atmosphere atmosphere;
    outer World.MyWorld world;
    
    parameter Boolean limit_alpha_dot = true;
    parameter SI.Angle alpha_dot_max = from_deg(360*5);
    
    Modelica.Blocks.Interfaces.RealOutput alpha(final quantity = "Angle", final unit = "rad", displayUnit = "deg") annotation(
      Placement(visible = true, transformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    SI.Velocity v_b[3] "Velocity relative to wind in body frame (body frame)";
    SI.Velocity v_w[3] "Wind speed at current position";
    SI.Velocity v_norm;
    parameter SI.Velocity v_small = 1e-5 "Prevent division by zero when velocity is too small";
    Modelica.Blocks.Interfaces.RealOutput beta(final quantity = "Angle", final unit = "rad", displayUnit = "deg") annotation(
      Placement(visible = true, transformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput alpha_dot(displayUnit = "deg", quantity = "Angle", unit = "rad") annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    v_w = atmosphere.windSpeed(world.altitude(frame_a.r_0));
    v_b = Frames.resolve2(frame_a.R, der(frame_a.r_0) - v_w);
    v_norm = Vectors.norm(v_b);
    if noEvent(abs(v_b[1]) > v_small) then
      alpha = atan(v_b[3] / v_b[1]);
    elseif noEvent(abs(v_b[3]) > abs(v_b[1])) then
      alpha = sign(v_b[3]) * pi / 2;
    else
      alpha = 0;
    end if;
    
    
    if noEvent(abs(der(alpha)) > alpha_dot_max and limit_alpha_dot) then
      alpha_dot = alpha_dot_max*sign(der(alpha));
    else
      alpha_dot = der(alpha);
    end if;
    
    if noEvent(abs(v_b[1]) > v_small) then
    beta = atan(v_b[2] / v_b[1]);
//sideslip = asin(v_b[2] / v_norm);
    elseif noEvent(abs(v_b[2]) > v_small) then
    beta = Modelica.Units.Conversions.from_deg(90 * sign(v_b[2]));
    else
    beta = 0;
    end if;
    frame_a.f = zeros(3);
    frame_a.t = zeros(3);
    annotation(
      Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Text(origin = {113, 44}, extent = {{-47, 16}, {47, -16}}, textString = "alpha"), Text(origin = {111, -95}, extent = {{-57, 15}, {57, -15}}, textString = "beta"), Text(origin = {118, -25}, extent = {{-60, 15}, {60, -15}}, textString = "alpha_dot")}));
  end AeroAnglesSensor;

  class AeroData
    type E = enumeration(:);
    replaceable type State = E;
  
    class ExternalAeroData
      extends ExternalObject;
  
      function constructor
        input String states_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_states.npz");
        input String coeffs_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/for006_coeffs.npz");
        input String state_names[State] = stateStrings();
        input String coeff_names[Coefficients] = coefficientStrings();
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
      output Real[Coefficients] coefficients;
    
      external getAeroCoefficients(aerodata, state, size(state, 1), coefficients, size(coefficients, 1)) annotation(
        Library = {"aerodata", "cnpy", "z"},
        Include = "#include \"ModelicaAeroData.h\"");
    end getData;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end AeroData;

  type Coefficients = enumeration(CN, CM, CA, CY, CLN, CLL, CNA, CMA, CYB, CLNB, CLLB, CNQ, CMQ, CAQ, CNAD, CMAD, CYR, CLNR, CLLR, CYP, CLNP, CLLP);

  function coefficientStrings
  output String array[Coefficients];
  algorithm
    for c in Coefficients loop
      array[c] := String(c);
    end for;
  end coefficientStrings;
  package Interfaces
    connector AeroStateInput
    input SI.Position altitude;
    input SI.MachNumber mach;
    input SI.Angle alpha(displayUnit = "deg");
    input SI.AngularVelocity alpha_dot(displayUnit = "deg/s");
    input SI.Angle beta(displayUnit = "deg");
    
    input SI.Velocity v[3];
    input SI.AngularVelocity w[3](each displayUnit = "deg/s");
    
    annotation(
        Diagram(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}),
        Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}));
    end AeroStateInput;

    connector AeroStateOutput
      output SI.Position altitude;
      output SI.MachNumber mach;
      output SI.Angle alpha(displayUnit = "deg");
      output SI.AngularVelocity alpha_dot(displayUnit = "deg/s");
      output SI.Angle beta(displayUnit = "deg");
      
      output SI.Velocity v[3];
      output SI.AngularVelocity w[3](each displayUnit = "deg/s");
      annotation(
        Diagram(graphics = {Polygon(origin = {20, 0}, fillColor = {135, 221, 245}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}),
        Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {135, 221, 245}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}));
    end AeroStateOutput;

    model ComposeAeroState
  AeroStateOutput out annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput alpha annotation(
        Placement(visible = true, transformation(origin = {-100, 90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput beta annotation(
        Placement(visible = true, transformation(origin = {-100, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput mach annotation(
        Placement(visible = true, transformation(origin = {-100, 22}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput altitude annotation(
        Placement(visible = true, transformation(origin = {-100, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput v[3] annotation(
        Placement(visible = true, transformation(origin = {-100, -56}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput w[3] annotation(
        Placement(visible = true, transformation(origin = {-100, -90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      out.alpha = alpha;
      out.beta = beta;
      out.altitude = altitude;
      out.mach = mach;
      out.v = v;
      out.w = w;
      annotation(
        Icon(graphics = {Line(origin = {9.59965, 45.1944}, points = {{-89.5996, 44.8056}, {90.4004, -45.1944}, {-89.5996, 10.8056}}), Line(origin = {9.50306, 0}, points = {{-89.5031, 20}, {90.4969, 0}, {-89.5031, -20}}), Line(origin = {10, -28}, points = {{-90, -28}, {90, 28}}), Line(origin = {10, -45}, points = {{-90, -45}, {90, 45}}), Text(origin = {4, -125}, extent = {{-160, 25}, {160, -25}}, textString = "a,b,m,h,v,w")}));
    end ComposeAeroState;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Interfaces;

  package Internal

    package Icons
      model AerodynamicsIcon
      equation
      
        annotation(
          Icon(graphics = {Polygon(origin = {4, 6}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, points = {{-58, 24}, {-74, 18}, {-80, 8}, {-76, -4}, {-66, -10}, {-52, -12}, {-36, -12}, {-22, -12}, {8, -12}, {42, -16}, {80, -24}, {54, -8}, {22, 6}, {-4, 16}, {-32, 22}, {-58, 24}}), Polygon(origin = {25, 17}, rotation = -90, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Line(origin = {-0.845794, -7.65421}, points = {{-88.5, 17}, {-80.5, 17}, {-76.5, 7}, {-64.5, -3}, {-38.5, -5}, {15.5, -5}, {49.5, -9}, {83.5, -17}, {99.5, -17}, {97.5, -17}}), Text(origin = {69, 42}, extent = {{-25, 26}, {25, -26}}, textString = "D"), Polygon(origin = {-17, 51}, fillPattern = FillPattern.Solid, points = {{-1, -39}, {5, -39}, {5, 23}, {13, 23}, {1, 43}, {-9, 23}, {-1, 23}, {-1, 17}, {-1, -39}}), Line(origin = {-0.690265, 17.9989}, points = {{-89, 6.00111}, {-85, 6.00111}, {-75, 18.0011}, {-55, 24.0011}, {-25, 22.0011}, {5, 16.0011}, {71, -11.9989}, {87, -23.9989}, {99, -23.9989}}), Line(origin = {-2, 11.9985}, points = {{-88, 6.00147}, {-82, 6.00147}, {-72, 18.0015}, {-52, 24.0015}, {-26, 22.0015}, {6, 16.0015}, {36, 4.00147}, {70, -11.9985}, {88, -23.9985}, {98, -23.9985}}), Text(origin = {11, 84}, extent = {{-25, 26}, {25, -26}}, textString = "L"), Line(origin = {0.327103, -17.5981}, points = {{-90, 20}, {-88, 20}, {-70, 0}, {-42, -2}, {16, -2}, {50, -6}, {82, -14}, {100, -14}})}));
      end AerodynamicsIcon;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Icons;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Internal;
end Aerodynamics;
