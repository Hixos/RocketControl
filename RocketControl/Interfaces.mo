within RocketControl;

package Interfaces
  connector AeroStateOutput
    output SI.Position altitude;
    output SI.MachNumber mach;
    output SI.Angle alpha(displayUnit = "deg");
    output SI.Angle beta(displayUnit = "deg");
    
    output SI.Velocity v[3];
    output SI.AngularVelocity w[3](each displayUnit = "deg/s");
    annotation(
      Diagram(graphics = {Polygon(origin = {20, 0}, fillColor = {135, 221, 245}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}),
      Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {135, 221, 245}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}));
  end AeroStateOutput;

  connector AeroStateInput
  input SI.Position altitude;
  input SI.MachNumber mach;
  input SI.Angle alpha(displayUnit = "deg");
  input SI.Angle beta(displayUnit = "deg");
  
  input SI.Velocity v[3];
  input SI.AngularVelocity w[3](each displayUnit = "deg/s");
  
  annotation(
      Diagram(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}),
      Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}));
  end AeroStateInput;

  model RealAeroState
  RocketControl.Interfaces.AeroStateOutput out annotation(
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
      Icon(graphics = {Line(origin = {9.59965, 45.1944}, points = {{-89.5996, 44.8056}, {90.4004, -45.1944}, {-89.5996, 10.8056}}), Line(origin = {9.50306, 0}, points = {{-89.5031, 20}, {90.4969, 0}, {-89.5031, -20}}), Line(origin = {10, -28}, points = {{-90, -28}, {90, 28}}), Line(origin = {10, -45}, points = {{-90, -45}, {90, 45}}), Text(origin = {4, -125}, extent = {{-160, 25}, {160, -25}}, textString = "a,b,m,h,v,w")}));end RealAeroState;

  connector MassPropertiesInput
  input SI.Mass m;
  input SI.Inertia I[3,3];
  annotation(
      Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {-40, -100}, {80, -20}, {80, 20}, {-40, 100}, {-80, 100}})}),
      Diagram(graphics = {Polygon(origin = {6, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, points = {{-40, 42}, {-40, -40}, {-16, -40}, {52, -8}, {52, 6}, {-16, 42}, {-40, 42}}), Text(extent = {{-140, -50}, {140, -90}}, textString = "%name")}));
  end MassPropertiesInput;

  connector MassPropertiesOutput
  output SI.Mass m;
  output SI.Inertia I[3,3];
  annotation(
      Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {-40, -100}, {80, -20}, {80, 20}, {-40, 100}, {-80, 100}})}),
      Diagram(graphics = {Polygon(origin = {6, 0}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, points = {{-40, 42}, {-40, -40}, {-16, -40}, {52, -8}, {52, 6}, {-16, 42}, {-40, 42}}), Text(extent = {{-140, -50}, {140, -90}}, textString = "%name")}));
  end MassPropertiesOutput;

  model AeroStateDemux
  RocketControl.Interfaces.AeroStateInput state annotation(
      Placement(visible = true, transformation(origin = {-98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput alpha annotation(
      Placement(visible = true, transformation(origin = {104, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {96, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput beta annotation(
      Placement(visible = true, transformation(origin = {104, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {96, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput altitude annotation(
      Placement(visible = true, transformation(origin = {104, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {96, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput mach annotation(
      Placement(visible = true, transformation(origin = {104, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {96, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput v[3] annotation(
      Placement(visible = true, transformation(origin = {104, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {96, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput w[3] annotation(
      Placement(visible = true, transformation(origin = {104, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {96, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
state.v = v;
  state.w = w;
  state.altitude = altitude;
  state.mach = mach;
  state.alpha = alpha;
  state.beta = beta;
  annotation(
      Icon(graphics = {Line(origin = {61.6213, -1.10634}, points = {{38.3787, 91.1063}, {-41.6213, 91.1063}, {-41.6213, -88.8937}, {42.3787, -88.8937}, {34.3787, -90.8937}}), Line(origin = {57, -52}, points = {{37, 0}, {-37, 0}}), Line(origin = {56, -18}, points = {{36, 0}, {-36, 0}}), Line(origin = {57, 16}, points = {{37, 0}, {-37, 0}}), Line(origin = {55, 54}, points = {{35, 0}, {-35, 0}}), Line(origin = {-40, 0}, points = {{-60, 0}, {60, 0}})}));end AeroStateDemux;

  package Debug
    model MassPropertiesMux
      RocketControl.Interfaces.MassPropertiesOutput out annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput m annotation(
        Placement(visible = true, transformation(origin = {-104, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-96, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput I[6] annotation(
        Placement(visible = true, transformation(origin = {-104, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-92, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      out.m = m;
      out.I[1, 1] = I[1];
      out.I[2, 2] = I[2];
      out.I[3, 3] = I[3];
      out.I[1, 2] = I[4];
      out.I[1, 3] = I[5];
      out.I[2, 3] = I[6];
      out.I[2, 1] = I[4];
      out.I[3, 1] = I[5];
      out.I[3, 2] = I[6];
      annotation(
        Icon(graphics = {Line(origin = {-52.397, 0.412098}, points = {{-48, 60}, {48, 60}, {48, -60}, {-40, -60}, {40, -60}}), Line(origin = {49, 0}, points = {{53, 0}, {-53, 0}}), Text(origin = {-68, 196.667}, extent = {{28, -126.667}, {-28, -88.6667}}, textString = "m"), Text(origin = {-68, 70.67}, extent = {{28, -126.67}, {-28, -88.67}}, textString = "I")}));
    end MassPropertiesMux;

    model MassPropertiesDemux
    RocketControl.Interfaces.MassPropertiesInput mass_in annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput m annotation(
        Placement(visible = true, transformation(origin = {94, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput I[6] annotation(
        Placement(visible = true, transformation(origin = {94, -38}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
    mass_in.m = m;
      mass_in.I[1, 1] = I[1];
      mass_in.I[2, 2] = I[2];
      mass_in.I[3, 3] = I[3];
      mass_in.I[1, 2] = I[4];
      mass_in.I[1, 3] = I[5];
      mass_in.I[2, 3] = I[6];
      mass_in.I[2, 1] = I[4];
      mass_in.I[3, 1] = I[5];
      mass_in.I[3, 2] = I[6];
    annotation(
        Icon(graphics = {Line(origin = {40, 0}, points = {{40, 60}, {-40, 60}, {-40, -60}, {40, -60}}), Line(origin = {-50, 0}, points = {{-50, 0}, {50, 0}}), Text(origin = {50, 192.667}, extent = {{28, -126.667}, {-28, -88.6667}}, textString = "m"), Text(origin = {50, 70.67}, extent = {{28, -126.67}, {-28, -88.67}}, textString = "I")}));end MassPropertiesDemux;

    model RealToDeflection
    RocketControl.Interfaces.FinDeflectionOutput finDeflectionOutput annotation(
        Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(visible = true, transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-98, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
finDeflectionOutput.deflection = u;
    annotation(
        Icon(graphics = {Polygon(origin = {0, 3}, fillColor = {102, 117, 134}, fillPattern = FillPattern.Solid, points = {{-66, -41}, {52, 55}, {66, 39}, {-54, -55}, {-56, -53}, {-66, -41}}), Line(origin = {64.96, 18.57}, points = {{5.04018, -18.5658}, {-2.95982, 19.4342}, {-4.95982, 13.4342}}, thickness = 0.75), Line(origin = {64, 36}, points = {{-2, 2}, {2, -2}}, thickness = 0.75), Text(origin = {84, 30}, extent = {{-18, 10}, {18, -10}}, textString = "u")}));end RealToDeflection;
  end Debug;

  partial model PartialConditionalEnablePort
  parameter Boolean useEnablePort = false "= true, if enable port is enabled" annotation(
      Evaluate = true,
      HideResult = true,
      choices(checkBox = true));
    parameter Boolean default_enabled = true annotation(
      Dialog(enable = not useEnablePort));
    Modelica.Blocks.Interfaces.BooleanInput enable annotation(
      Placement(visible = true, transformation(origin = {0, 106}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 92}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  equation
    if not useEnablePort then
      enable = default_enabled;
    end if;
    annotation(
      Icon(graphics = {Text(origin = {-85.999, 138}, lineColor = {128, 128, 128}, extent = {{99.9988, -29}, {135.999, -58}}, textString = "e")}));
  end PartialConditionalEnablePort;

  connector FinDeflectionOutput
    output SI.Angle deflection (displayUnit = "deg");
  annotation(
      Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {0, 170, 0}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}}, smooth = Smooth.Bezier)}));
  end FinDeflectionOutput;

  connector FinDeflectionInput
    input NonSI.Angle_deg deflection (displayUnit = "deg");
  annotation(
      Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {170, 255, 127}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}}, smooth = Smooth.Bezier)}));
  end FinDeflectionInput;

  expandable connector AvionicsBus
    //  SI.Velocity v;
    Boolean liftoff;
    SI.Position x_meas[3];
    SI.Acceleration a_meas[3];
    SI.Acceleration a_est[3];
    RocketControl.Types.AngularVelocity_degs[3] w_meas;
    RocketControl.Types.AngularVelocity_degs[3] w_est;
    RocketControl.Types.NanoTesla b_meas[3];
    SI.Pressure p_meas;
    Real q_est[4];
    annotation(
      Icon(graphics = {Ellipse(origin = {0, 0.0100002}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-60, 59.99}, {60, -59.99}}, endAngle = 360), Ellipse(origin = {-0.0400009, -0.360001}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-29.96, 30.36}, {29.96, -30.36}}, endAngle = 360), Ellipse(fillColor = {255, 255, 0}, fillPattern = FillPattern.Solid, extent = {{-10, 10}, {10, -10}}, endAngle = 360)}));
  end AvionicsBus;

  model PartialAvionicsBusPort
  RocketControl.Interfaces.AvionicsBus bus annotation(
      Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {101, 99}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
  equation

    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end PartialAvionicsBusPort;

  package Internal
    package Icons
      model Estimator
      equation

        annotation(
          Icon(graphics = {Rectangle(fillColor = {252, 255, 252}, fillPattern = FillPattern.Solid, extent = {{-90, 90}, {90, -90}}, radius = 50), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name"), Line(origin = {52.2533, 55.5403}, points = {{-18, -18}, {18, 18}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Line(origin = {25.5114, -29.1575}, rotation = -90, points = {{-18, -18}, {24, 24}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name"), Line(origin = {-43.1155, -11.4371}, rotation = 180, points = {{-18, -18}, {36, -18}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Polygon(origin = {-46, -40}, fillColor = {91, 91, 91}, fillPattern = FillPattern.Solid, points = {{-2, -12}, {8, -22}, {22, 12}, {12, 22}, {-22, 8}, {-2, -12}}), Polygon(origin = {-15, -9}, fillColor = {155, 183, 193}, fillPattern = FillPattern.Solid, points = {{-43, -33}, {25, 35}, {45, 45}, {35, 25}, {-33, -43}, {-37, -39}, {-43, -33}})}));
      end Estimator;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Icons;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Internal;
end Interfaces;
