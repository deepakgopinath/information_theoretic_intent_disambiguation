function [ T ] = MatrixExp6( stheta )

%Usage:
%input - exponential coordinates 6 by 1
%output - Transformation matrix represented by the exponential coordinates

T = [];
if(size(stheta,2) > 1) %Make it a column vector if a row vector
    stheta = stheta';
end

if(size(stheta,2) > 1) % if a matrix
    disp('stheta is not a vector');
elseif(size(stheta, 1) ~= 6)
    disp('stheta is not a 6d vector')
else
     if(sum(stheta == [0,0,0,0,0,0]') == 6) %trivial case. 
         T = eye(4,4);
     elseif(sum(stheta(1:3) == [0,0,0]') == 3) % if the w vector is 0
        R = eye(3);
        T = zeros(4,4);
        vprime = stheta(4:6)/norm(stheta(4:6));
        theta = norm(stheta(4:6));
        T(1:3,1:3) = R;
        T(4,:) = [0,0,0,1];
        T(1:3, 4) = vprime*theta;
    else
        w = stheta(1:3)/norm(stheta(1:3)); %unit vector of omega
        vprime = stheta(4:6)/norm(stheta(1:3)); % v component of the screw axis
        theta = norm(stheta(1:3));
        
        ssMatrix = VecToso3(w); %[w]
        R = eye(3) + sin(theta)*(ssMatrix) + (1 - cos(theta))*(ssMatrix^2); %e^[w]theta - Rodrguez Formula
        
        T = zeros(4,4);
        T(1:3,1:3) = R;
        T(4,:) = [0,0,0,1];
        T(1:3, 4) = (eye(3)*theta + (1 - cos(theta))*ssMatrix + (theta - sin(theta))*(ssMatrix^2))*(vprime);
    end
end
end

