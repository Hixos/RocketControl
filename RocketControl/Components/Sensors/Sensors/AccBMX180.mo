within RocketControl.Components.Sensors.Sensors;

model AccBMX180
  extends RealSensors.RealAccelerometer(acc_max = 156.96, bias = {0.02, 0.04, -0.03}, bits = 16, sigmaBiasInstability = 0.01, sigmaNoise = 0.1, noisy = true, biased = true, limited = true, quantized = true);
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end AccBMX180;
