within RocketControl.Tests.Math;

model ctrb
parameter Real A[:, size(A,1)] = diagonal({1,2});
parameter Real B[size(A,1),:] = [1; 0];
Real C[size(A,1),size(A,1)*size(B,2)];
equation
C = RocketControl.Math.Matrices.ctrb(A,B);
annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end ctrb;
