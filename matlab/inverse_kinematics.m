function [J11, J12, J21, J22] = inverse_kinematics(x, y, L1, L2)
    % There are two solutions to J2
    J21 = pi - acos((-x^2 - y^2 + L1^2 + L2^2) / (2*L1*L2)); 
    J22 = -J21;

    % sometimes J2 is imaginary. I think and hope this is because of numerical
    % error
    if imag(J21) > 1e-5 | imag(J22) > 1e-5
        fprintf("Invalid position\n")
        J11 = NaN;
    elseif imag(J21) < 1e-5 & imag(J22) < 1e-5
        
        J21 = real(J21);
        J22 = real(J22);

        s21 = sin(J21);
        c21 = cos(J21);
        s22 = sin(J22);
        c22 = cos(J22);

        a1 = L1+L2*c21;
        b1 = L2*s21;
        s11 = (y*a1-b1*x)/(b1^2+a1^2);
        c11 = (x*a1+b1*y)/(b1^2+a1^2);
        J11 = atan2(s11, c11);

        a2 = L1+L2*c22;
        b2 = L2*s22;
        s12 = (y*a2-b2*x)/(b2^2+a2^2);
        c12 = (x*a2+b2*y)/(b2^2+a2^2);
        J12 = atan2(s12, c12);
    end

end