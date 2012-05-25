function q = arm_ik(H,a3)

q = [0,0,0]';

th13 = atan2(H(2,1),H(1,1));
q(1) = atan2(H(1,4) - a3*cos(th13),a3*sin(th13) - H(2,4));
q(2) = sqrt((H(1,4) - a3*cos(th13))^2 + (H(2,4) - a3*sin(th13))^2);
q(3) = th13 - q(1);

end
