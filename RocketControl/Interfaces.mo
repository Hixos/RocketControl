within RocketControl;

package Interfaces
  connector MassPropertiesInput
    input SI.Mass m;
    input SI.Inertia I[3, 3];
    annotation(
      Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {-40, -100}, {80, -20}, {80, 20}, {-40, 100}, {-80, 100}})}),
      Diagram(graphics = {Polygon(origin = {6, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, points = {{-40, 42}, {-40, -40}, {-16, -40}, {52, -8}, {52, 6}, {-16, 42}, {-40, 42}}), Text(extent = {{-140, -50}, {140, -90}}, textString = "%name")}));
  end MassPropertiesInput;

  connector MassPropertiesOutput
    output SI.Mass m;
    output SI.Inertia I[3, 3];
    annotation(
      Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {-40, -100}, {80, -20}, {80, 20}, {-40, 100}, {-80, 100}})}),
      Diagram(graphics = {Polygon(origin = {6, 0}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, points = {{-40, 42}, {-40, -40}, {-16, -40}, {52, -8}, {52, 6}, {-16, 42}, {-40, 42}}), Text(extent = {{-140, -50}, {140, -90}}, textString = "%name")}));
  end MassPropertiesOutput;

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

  //expandable connector AvionicsBus

  expandable connector AvionicsBus
    //  SI.Velocity v;
    Boolean liftoff;
    SI.Position x_meas[3];
    SI.Velocity v_meas[3];
    SI.Velocity x_est[3];
    SI.Velocity v_est[3];
    SI.Acceleration a_meas[3];
    SI.Acceleration a_est[3];
    RocketControl.Types.AngularVelocity[3] w_meas;
    RocketControl.Types.AngularVelocity[3] w_est;
    SI.MagneticFluxDensity b_meas[3](each displayUnit = "nT");
    SI.Pressure p_meas;
    SI.Angle control_cmd[3];
    Real q_est[4];
    annotation(
      Icon(graphics = {Ellipse(origin = {0, 0.0100002}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-60, 59.99}, {60, -59.99}}, endAngle = 360), Ellipse(origin = {-0.0400009, -0.360001}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-29.96, 30.36}, {29.96, -30.36}}, endAngle = 360), Ellipse(fillColor = {255, 255, 0}, fillPattern = FillPattern.Solid, extent = {{-10, 10}, {10, -10}}, endAngle = 360)}));
  end AvionicsBus;

  model ComposeMassProperties
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
  end ComposeMassProperties;

  model AvionicsBusDemux
    RocketControl.Components.Interfaces.AvionicsBus avionicsBus annotation(
      Placement(visible = true, transformation(origin = {0, 94}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {32, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput pos annotation(
      Placement(visible = true, transformation(origin = {-90, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {-270, -32}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealOutput vel annotation(
      Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {272, -32}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  equation
    connect(avionicsBus.x_meas, pos) annotation(
      Line(points = {{0, 94}, {0, 40}, {-90, 40}, {-90, -10}}));
    connect(avionicsBus.v_est, vel) annotation(
      Line(points = {{0, 94}, {0, 40}, {-50, 40}, {-50, -10}}));
    annotation(
      Icon(graphics = {Rectangle(origin = {-259.95, 9.05}, fillPattern = FillPattern.Solid, extent = {{-299.43, 10.3}, {299.43, -10.3}}), Text(origin = {-272, 69}, rotation = -90, fillColor = {85, 85, 85}, extent = {{-38, 13}, {38, -13}}, textString = "x_meas")}));
  end AvionicsBusDemux;

  package Debug
  model ComposeMassProperties
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
  end ComposeMassProperties;
  
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Debug;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Interfaces;
