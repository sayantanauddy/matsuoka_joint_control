
% Initialize state variables
setGlobal();

y = [];
t=0.0;
while t<=50
    t = t + 0.01;
    tau = matsuoka_torque(0,t,0,0);
    y = [y tau(1)];
end

x = [0:0.01:50];
x(1000:end) = [];
y(1000:end) = [];

hold on;
plot(x,y);
legend('output of oscillator - torque');
xlabel('time');
ylabel('torque');
hold off;

