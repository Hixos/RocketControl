within RocketControl;

package Aerodynamics
extends Icons.AerodynamicsIcon;
partial model PartialAerodynamicForce
extends Icons.AerodynamicsIcon;
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
  extends Icons.AerodynamicsIcon;
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

  type Coefficients = enumeration(CN, CM, CA, CY, CLN, CLL, CNA, CMA, CYB, CLNB, CLLB, CNQ, CMQ, CAQ, CNAD, CMAD, CYR, CLNR, CLLR, CYP, CLNP, CLLP);

  function coefficientStrings
  extends Modelica.Icons.Function;
  output String array[Coefficients];
  algorithm
    for c in Coefficients loop
      array[c] := String(c);
    end for;
  end coefficientStrings;

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
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Aerodynamics;
