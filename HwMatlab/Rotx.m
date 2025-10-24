function R = Rotx(theta)
    % Rotx - Computes the rotation matrix for a rotation about the x-axis
        % Inputs:
    %    theta - Rotation angle in radians
      % Outputs:
    %    R - 3x3 rotation matrix
         R = [1,         0,          0;
         0, cos(theta), -sin(theta);
         0, sin(theta),  cos(theta)];
end
