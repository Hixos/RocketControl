within RocketControl.Components.Sensors.Sensors;

model GyroBMX180
  extends RealSensors.RealGyroscope(bias = {0.00174532925199433, -0.0008726646259971648, -0.0003490658503988659}, bits = 16, rate_max = 4.363323129985824, sigmaARW = 0.008726646259971648, sigmaRRW = 0.0174532925199433, noisy = true, biased = true, limited = true, quantized = true);
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end GyroBMX180;
