mdl_twolink;
%onelink.links(1).m=20;
setGlobal;
[t,q] = twolink.nofriction.fdyn(20,@matsuoka_torque);
q1 = arrayfun(@(x) modpi(x), q(1:end,1));
q2 = arrayfun(@(x) modpi(x), q(1:end,2));
hold on;
plot(t,q1);
plot(t,q2);
legend('q1-controlled');
xlabel('time');
ylabel('joint angle (rad)');
hold off;


