within RocketControl.Rockets.Lynx.GNC.Sensors;

model RealSensors
  extends Rockets.Internal.PartialSensorsPackage;
  
  outer World.SimOptions opt;
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Sensors.GyroBMX180 gyroBMX180(biased = true, fixedLocalSeed = {21, 201, 2001}, limited = true, noisy = true, quantized = true, rate_max(displayUnit = "rad/s"), samplePeriodMs = opt.samplePeriodMs, sigmaARW(displayUnit = "rad/s"), sigmaRRW(displayUnit = ""))  annotation(
    Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Sensors.AccBMX180 accBMX180(biased = true, fixedLocalSeed = {22, 202, 2002}, limited = true, noisy = true, quantized = true, samplePeriodMs = opt.samplePeriodMs)  annotation(
    Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Sensors.MagBMX160 magBMX160(biased = true, fixedLocalSeed = {23, 203, 2003}, limited = true, noisy = true, quantized = true, samplePeriodMs = opt.samplePeriodMs, sigmaNoise(displayUnit = "T"))  annotation(
    Placement(visible = true, transformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Sensors.BaroMS5803 baroMS5803(biased = true, fixedLocalSeed = 24, limited = true, noisy = true, quantized = true, samplePeriodMs = opt.samplePeriodMs)  annotation(
    Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Sensors.GNSSUbloxM7N gNSSUbloxM7N(fixedLocalSeed = {2511, 20511, 200511},noisy = true, samplePeriodMs = opt.samplePeriodMs)  annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Clocked.VectorSample vectorSample(n = 4)  annotation(
    Placement(visible = true, transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.Deflection2Control deflection2Control annotation(
    Placement(visible = true, transformation(origin = {30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(gyroBMX180.frame_a, frame_a) annotation(
    Line(points = {{-10, 80}, {-60, 80}, {-60, 0}, {-100, 0}}));
  connect(accBMX180.frame_a, frame_a) annotation(
    Line(points = {{-10, 50}, {-60, 50}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(magBMX160.frame_a, frame_a) annotation(
    Line(points = {{-10, 20}, {-60, 20}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(baroMS5803.frame_a, frame_a) annotation(
    Line(points = {{-10, -10}, {-60, -10}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(gNSSUbloxM7N.frame_a, frame_a) annotation(
    Line(points = {{-10, -40}, {-60, -40}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(gyroBMX180.w, bus.w_meas) annotation(
    Line(points = {{10, 80}, {60, 80}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(accBMX180.a, bus.a_meas) annotation(
    Line(points = {{10, 50}, {60, 50}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(magBMX160.b, bus.b_meas) annotation(
    Line(points = {{10, 20}, {60, 20}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(baroMS5803.p, bus.p_meas) annotation(
    Line(points = {{10, -10}, {60, -10}, {60, 0}, {100, 0}}, color = {0, 0, 127}));
  connect(gNSSUbloxM7N.x, bus.x_meas) annotation(
    Line(points = {{12, -36}, {60, -36}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gNSSUbloxM7N.v, bus.v_meas) annotation(
    Line(points = {{12, -44}, {60, -44}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(frame_a, vectorSample.u) annotation(
    Line(points = {{-100, 0}, {-60, 0}, {-60, -70}, {-22, -70}}));
  connect(vectorSample.y, deflection2Control.u) annotation(
    Line(points = {{2, -70}, {18, -70}}, color = {0, 0, 127}, thickness = 0.5));
  connect(deflection2Control.control, bus.control_position_meas) annotation(
    Line(points = {{42, -70}, {60, -70}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end RealSensors;
