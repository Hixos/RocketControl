within RocketControl.Components.Sensors.Sensors;

model GNSSUbloxM7N
  extends SensorModels.RealGNSS(sigmaNoise_vxy = 3, sigmaNoise_vz = 5, sigmaNoise_xy = 15, sigmaNoise_z = 30, sin_error_amplitude = {30, 30, 50}, sin_error_freq = 0.005, sin_error_phase = from_deg({20, 30, 40}), noisy = true);
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end GNSSUbloxM7N;
