within RocketControl.Interfaces;

expandable connector AvionicsBus
  extends Modelica.Icons.SignalBus;
  //  SI.Velocity v;
  Boolean liftoff;
  Boolean roll_guidance;
  Boolean velocity_guidace;
  Boolean apogee_guidance;
  Boolean drogue_deploy;
  Boolean main_deploy;
  Boolean gps_fix;
  SI.Position x_meas[3];
  SI.Velocity v_meas[3];
  SI.Velocity x_est[3];
  SI.Velocity v_est[3];
  SI.Acceleration a_meas[3];
  RocketControl.Types.AngularVelocity[3] w_meas;
  RocketControl.Types.AngularVelocity[3] w_est;
  SI.MagneticFluxDensity b_meas[3](each displayUnit = "nT");
  SI.Pressure p_meas;
  
  SI.Angle fin_setpoint[4];
  SI.Angle fin_true_position[4];
  SI.Angle control_position_meas[4];
  SI.AngularAcceleration w_dot[3];
  SI.Velocity v_ref[3];
   RocketControl.Types.AngularVelocity[4] cpos_dot;
  Real q_est[4];
  annotation(
    Icon);
end AvionicsBus;
