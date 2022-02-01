within RocketControl.Components.Sensors.Sensors;

model BaroMS5803
  extends RealSensors.RealBarometer(bias(displayUnit = "Pa") = 300, bits = 24, p_max(displayUnit = "Pa") = 110000, p_min(displayUnit = "Pa") = 999.9999999999999, sigmaNoise(displayUnit = "Pa") = 50, noisy = true, biased = true, limited = true, quantized = true);
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end BaroMS5803;
