within RocketControl.Tests.Math;

model sdaCare
  parameter Real A[5, 5] = [0, 0, 1.0000e+00, 0, 0; 0, 0, 0, 1.0000e+00, 0; -6.5400e+01, 6.5400e+01, -3.2700e+00, 3.2700e+00, 0; 1.9620e+03, -6.2784e+04, 9.8100e+01, -9.8100e+01, 0; -1.0000e+00, 0, 0, 0, 0];
  parameter Real B[5, 1] = [0; 0; 1.66666666666667e-03; -5e-02; 0];
  parameter Real Q[5, 5] = diagonal({0, 0, 0, 0, 1});
  parameter Real R[1, 1] = [0.001];
  parameter Real P[5, 5] = RocketControl.Math.sdaCare(A, B, R, Q);
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end sdaCare;
