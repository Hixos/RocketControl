within RocketControl.Blocks.Icons;

model QuaternionIcon
equation

  annotation(
    Icon(graphics = {Rectangle(lineColor = {0, 170, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {0, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end QuaternionIcon;
