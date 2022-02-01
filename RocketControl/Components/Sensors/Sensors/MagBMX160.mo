within RocketControl.Components.Sensors.Sensors;

model MagBMX160
  extends RealSensors.RealMagnetometer(b_max = 2500000 * 1e-9, bias = 250 * 1e-9, bits = 16, misalignement = {from_deg(0.2), from_deg(-0.1), from_deg(0.3)}, sigmaNoise = 1.000000000000001e-08, noisy = true, biased = true, limited = true, quantized = true);
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end MagBMX160;
