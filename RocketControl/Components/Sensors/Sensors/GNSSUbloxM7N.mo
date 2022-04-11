within RocketControl.Components.Sensors.Sensors;

model GNSSUbloxM7N
  extends SensorModels.RealGNSS(sigmaNoise_vxy = 1, sigmaNoise_vz = 1.5, sigmaNoise_xy = 5, sigmaNoise_z = 8, sin_error_amplitude = {5, 5, 10}, sin_error_freq = 0.005, sin_error_phase = from_deg({20, 70, 130}), noisy = true);
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end GNSSUbloxM7N;
