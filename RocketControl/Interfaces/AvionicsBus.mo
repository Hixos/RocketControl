within RocketControl.Interfaces;

expandable connector AvionicsBus
  //  SI.Velocity v;
  Boolean liftoff;
  SI.Position x_meas[3];
  SI.Velocity v_meas[3];
  SI.Velocity x_est[3];
  SI.Velocity v_est[3];
  SI.Acceleration a_meas[3];
  RocketControl.Types.AngularVelocity[3] w_meas;
  RocketControl.Types.AngularVelocity[3] w_est;
  SI.MagneticFluxDensity b_meas[3](each displayUnit = "nT");
  SI.Pressure p_meas;
  SI.Angle control_cmd[4];
  SI.Angle fin_position[4];
  Real q_est[4];
  annotation(
    Icon(graphics = {Ellipse(fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Ellipse(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-60, 60}, {60, -60}}), Ellipse(fillColor = {255, 255, 0}, fillPattern = FillPattern.Solid, extent = {{-40, 40}, {40, -40}})}));
end AvionicsBus;
