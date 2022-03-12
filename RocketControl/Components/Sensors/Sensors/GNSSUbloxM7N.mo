within RocketControl.Components.Sensors.Sensors;

model GNSSUbloxM7N
  extends SensorModels.RealGNSS(sigmaNoise_vxy = 2, sigmaNoise_vz = 3, sigmaNoise_xy = 10, sigmaNoise_z = 15, sin_error_amplitude = {15, 15, 30}, sin_error_freq = 0.005, sin_error_phase = from_deg({20, 70, 130}), noisy = true);
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end GNSSUbloxM7N;
