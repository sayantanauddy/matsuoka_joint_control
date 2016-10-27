mdl_twolink;
setGlobal;
[t,q] = twolink.fdyn(50,@matsuoka_torque);
hold on;
plot(t,q);
legend('q1-controlled','q2-no torque');
xlabel('time');
ylabel('joint angle (rad)');
hold off;

