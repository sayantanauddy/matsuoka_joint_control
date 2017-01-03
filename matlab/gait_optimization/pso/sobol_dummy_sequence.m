
% Script to generate a dummy plot using a Sobol sequence of 1000 2D points

% Plot using Sobol sequence
p = sobolset(2,'Skip',1e3,'Leap',1e2);
p = scramble(p,'MatousekAffineOwen');
sobel_points = net(p,1000);
x_s=sobel_points(:,1)
y_s=sobel_points(:,2)
figure;
hold on;
plot(1);
scatter(x_s,y_s,'.','r');
axis([0 1 0 1]);
hold off;

% Plot using uniform distribution
uniform_points = zeros(1000,2);
for i=1:1000
    for j=1:2
        uniform_points(i,j) = 0.0 + (1.0 - 0.0)*rand(1,1);
    end
end
x_u=uniform_points(:,1);
y_u=uniform_points(:,2);
figure;
hold on;
plot(2);
scatter(x_u,y_u,'.','b');
axis([0 1 0 1]);
hold off;
