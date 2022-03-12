within RocketControl.Components.Sensors.SensorModels;

model RealFinPositionSensor
  extends Modelica.Icons.RoundSensor;
  
  parameter Integer samplePeriodMs(min = 1) "Sample period in milliseconds" annotation(
    Dialog(group = "Sampling and noise"));
  parameter Boolean noisy = false "= true, if output should be superimposed with noise" annotation(
    Evaluate = true,
    choices(checkBox = true),
    Dialog(group = "Sampling and noise"));
  parameter Boolean biased = false "= true, if output should be biased" annotation(
    Evaluate = true,
    choices(checkBox = true),
    Dialog(group = "Bias"));
  parameter Boolean limited = false "= true, if output is limited" annotation(
    Evaluate = true,
    choices(checkBox = true),
    Dialog(group = "Limiting and quantization"));
  parameter Boolean quantized = false "= true, if output quantization effects included" annotation(
    Evaluate = true,
    choices(checkBox = true),
    Dialog(enable = limited, group = "Limiting and quantization"));
    
  parameter SI.Angle bias[4] "Measurement bias for each axis" annotation(
    Dialog(enable = biased, group = "Bias"));
    
  parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
    Dialog(enable = limited and quantized, group = "Limiting and quantization"));
    
  parameter Integer fixedLocalSeed[4] = {10, 100, 1000, 10000} "Local seed for each of the fin sensors" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
    
  parameter SI.Angle sigma_noise "Angle standard deviation" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  
    parameter SI.Angle angle_max(displayUnit = "deg") "Angle measurement upper limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter SI.Angle angle_min(displayUnit = "deg") = -angle_max "Angle measurement lower limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  
  Modelica.Blocks.Interfaces.RealOutput control_meas[4](each displayUnit = "deg", each final quantity = "Angle", each final unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
  Modelica.Blocks.Interfaces.RealInput fin_pos_true[4](each displayUnit = "deg", each final quantity = "Angle", each final unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    
  RocketControl.Components.Sensors.TrueSensors.TrueFinPositionSensor trueFinPositionSensor annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleY(bias = bias[1], biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[1], sigma = sigma_noise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, samplePeriodMs = samplePeriodMs, yMax = angle_max, yMin = angle_min) annotation(
    Placement(visible = true, transformation(origin = {40, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    
    RocketControl.Components.Sensors.Internal.ADeffects sampleP(bias = bias[2], biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[2], sigma = sigma_noise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, samplePeriodMs = samplePeriodMs, yMax = angle_max, yMin = angle_min) annotation(
    Placement(visible = true, transformation(origin = {40, 20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    
      RocketControl.Components.Sensors.Internal.ADeffects sampleR(bias = bias[3], biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[3], sigma = sigma_noise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, samplePeriodMs = samplePeriodMs, yMax = angle_max, yMin = angle_min) annotation(
    Placement(visible = true, transformation(origin = {40, -20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    
      RocketControl.Components.Sensors.Internal.ADeffects sampleS(bias = bias[4], biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[4], sigma = sigma_noise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, samplePeriodMs = samplePeriodMs, yMax = angle_max, yMin = angle_min) annotation(
    Placement(visible = true, transformation(origin = {40, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
equation
  connect(trueFinPositionSensor.control_meas[1], sampleY.u) annotation(
    Line(points = {{-39, 0}, {0, 0}, {0, 60}, {32, 60}}, color = {0, 0, 127}));
  connect(trueFinPositionSensor.control_meas[2], sampleP.u) annotation(
    Line(points = {{-39, 0}, {0, 0}, {0, 20}, {32, 20}}, color = {0, 0, 127}));
  connect(trueFinPositionSensor.control_meas[3], sampleR.u) annotation(
    Line(points = {{-39, 0}, {0, 0}, {0, -20}, {32, -20}}, color = {0, 0, 127}));
  connect(trueFinPositionSensor.control_meas[4], sampleS.u) annotation(
    Line(points = {{-39, 0}, {0, 0}, {0, -60}, {32, -60}}, color = {0, 0, 127}));
  connect(sampleY.y, control_meas[1]) annotation(
    Line(points = {{46, 60}, {80, 60}, {80, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(sampleP.y, control_meas[2]) annotation(
    Line(points = {{46, 20}, {80, 20}, {80, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(sampleR.y, control_meas[3]) annotation(
    Line(points = {{46, -20}, {80, -20}, {80, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(sampleS.y, control_meas[4]) annotation(
    Line(points = {{46, -60}, {80, -60}, {80, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(fin_pos_true, trueFinPositionSensor.fin_pos_true) annotation(
    Line(points = {{-120, 0}, {-62, 0}}, color = {0, 0, 127}, thickness = 0.5));

annotation(
    Icon(graphics = {Line(origin = {85, 0}, points = {{-15, 0}, {15, 0}}), Line(origin = {-85, 0}, points = {{-15, 0}, {15, 0}}), Text(origin = {-2, -200}, lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(origin = {1, -38}, extent = {{-43, 18}, {43, -18}}, textString = "delta")}));
end RealFinPositionSensor;
