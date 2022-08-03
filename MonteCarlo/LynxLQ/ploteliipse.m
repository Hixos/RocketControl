close all
[c, a, b, angle, s, R] = ellipse_data(impacts_ol, 0.95);
adeg = rad2deg(angle)

figure
scatter(impacts_ol(:,2), impacts_ol(:,1));
hold on
drawEllipse(c, a, b, s, R);

function drawEllipse(c, a, b, s, R)
vx = linspace(-a*sqrt(s), a*sqrt(s), 1000);
vy = b*sqrt(s-(vx/a).^2);

P = [vx, flip(vx);
     vy, -vy];
 
for i=1:size(P, 2)
    P(:,i) = R*P(:,i) + c';
end
 
plot(P(2,:), P(1,:));
end

function [c, a, b, angle, s, R] = ellipse_data(data, conf)
c = mean(data);
data = data - c;
C = cov(data);
[V, D] = eig(C);
sigma = sqrt(diag(D));
a = sigma(1);
b = sigma(2);
[~, imax] = max(diag(D));
angle = atan2(V(2,imax), V(1,imax));
s = chi2inv(conf, 2);
R = V;
end