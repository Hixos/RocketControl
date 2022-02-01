within RocketControl.World;

model SimOptions
  parameter Integer samplePeriodMs = 20;

  annotation(defaultComponentPrefixes="inner",
    Icon(graphics = {Ellipse(fillColor = {255, 164, 250}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-5, 10}, extent = {{-81, 56}, {81, -56}}, textString = "opt")}));
end SimOptions;
