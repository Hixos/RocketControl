within RocketControl.Components.Sensors.Sensors;

model GyroBMX180
  extends SensorModels.RealGyroscope(bias = from_deg({0.2,0.1,-0.25}), bits = 16, rate_max = 4.363323129985824, sigmaARW = from_deg(0.3), sigmaRRW = from_deg(0.1), noisy = true, biased = true, limited = true, quantized = true);
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end GyroBMX180;
