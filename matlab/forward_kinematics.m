function [x,y] = forward_kinematics(J1, J2, L1, L2)
    x = cos(J1)*L1 + cos(J1 + J2)*L2;
    y = sin(J1)*L1 + sin(J1 + J2)*L2;
    
    if ~isreal(x) | ~isreal(y)
       fprintf("shajse\n"); 
    end
    
end