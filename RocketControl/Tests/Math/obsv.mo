within RocketControl.Tests.Math;

model obsv
parameter Real A[:, size(A,1)] = diagonal({1,2});
parameter Real C[:,size(A,1)] = [1, 0];
Real O[size(A,1)*size(C,1),size(A,1)];
equation
O = RocketControl.Math.Matrices.obsv(A,C);
annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end obsv;
