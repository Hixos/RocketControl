within RocketControl.Interfaces;

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
