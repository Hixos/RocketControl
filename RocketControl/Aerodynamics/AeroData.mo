within RocketControl.Aerodynamics;

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
