% This code makes sure that inverse kinematics and forward kinematics
% match. It takes a joint angle for J1 and J2 and calculates the x,y it
% corresponds to, using FK. Then calculate the angles again using IK. The
% resulting angles should be the same as J1 and J2. It then loops over all
% of the possible joint values and checks this

clc; clear all;
L1 = 1;
L2 = 2;
eps = 1e-1;

% Test Q1
test_quadrant(0, pi/2, L1, L2, eps)
% Test Q2
test_quadrant(pi/2, pi, L1, L2, eps)
% Test Q3
test_quadrant(-pi/2, -pi, L1, L2, eps)
test_quadrant(pi, 3*pi/4, L1, L2, eps)
% Test Q4
test_quadrant(0, -pi/2, L1, L2, eps)
test_quadrant(3*pi/4, 2*pi, L1, L2, eps)

fprintf("Passed all tests\n");


function test_quadrant(theta_min, theta_max, L1, L2, eps)
    for J1 = theta_min:0.01:theta_max
        for J2 = theta_min:0.01:theta_max
            [x,y] = forward_kinematics(J1, J2, L1, L2);
            [J11, J12, J21, J22] = inverse_kinematics(x,y,L1,L2);

            if isnan(J11) || isnan(J12) || isnan(J21) || isnan(J22)
                fprintf("Shajse\n")
            end


            if abs(sin(J1)-sin(J11)) > eps && abs(sin(J1)-sin(J12)) > eps % sin to avoid problem of -pi != pi, although it is
                fprintf("Shajse\n")
            end
            if abs(sin(J2)-sin(J21)) > eps && abs(sin(J2)-sin(J22)) > eps
                fprintf("Shajse\n")
            end
        end
    end

end