function [ stheta ] = MatrixLog6( T )

%Usage:
%Input - Transformation Matrix - SE(3)
%Output - Exponential coordinates corresponding to Transformation Matrix -
%6 by 1


stheta = [];
epsilon = 10^(-5);
if (numel(size(T)) ~= 2 || size(T,1) ~= size(T,2))
    disp('T is not a valid transformation matrix either because it has more than 2 dimensions or its not square');
elseif(size(T,1) ~= 4)
    disp('T is not 4 by 4');
elseif( sum(T(4,:) == [0,0,0,1]) ~= 4)
    disp('T is not a valid transformation matrix because last row is not [0,0,0,1]')
elseif(CheckIfIdentity(T(1:3,1:3)'*T(1:3,1:3)) == false || (abs(det(T(1:3,1:3))) - 1) > epsilon)
    disp('T is not a valid transformation matrix because the R component is not SO(3)');
else
    R = T(1:3,1:3);
    if(CheckIfIdentity(R) == true && sum(T(1:3,4) == [0,0,0]') == 3)
        stheta = zeros(6,1);
    elseif(CheckIfIdentity(R) == true)
        omega = zeros(3,1);    %no rotation. 
        v = T(1:3, 4)/norm(T(1:3,4));
        theta = norm(T(1:3,4));
        stheta = theta*[omega;v];
    elseif(trace(R) == -1)
        theta = pi;
        if((1 + R(3,3)) ~= 0)
            omega = (1/sqrt(2*(1+R(3,3))))*[R(1,3);R(2,3);1+R(3,3)]; %unit vector
        elseif((1 + R(2,2)) ~= 0)
            omega = (1/sqrt(2*(1+R(2,2))))*[R(1,2);1+R(2,2);R(3,2)];
        elseif((1 + R(1,1)) ~= 0)
            omega = (1/sqrt(2*(1+R(1,1))))*[1+R(1,1);R(2,1);R(3,1)];
        end
        ssMatrix = VecToso3(omega);
        GInv = (1/theta)*eye(3) - (1/2)*ssMatrix + ((1/theta) - (1/2)*cot(theta/2))*(ssMatrix^2);
        v = GInv*T(1:3, 4);
        stheta = theta*[omega;v];
    else
        theta = acos((trace(R) - 1)/2);
        ssMatrix = (1/2*sin(theta))*(R - R');
        omega = so3ToVec(ssMatrix);
        GInv = (1/theta)*eye(3) - (1/2)*ssMatrix + ((1/theta) - (1/2)*cot(theta/2))*(ssMatrix^2);
        v = GInv*T(1:3,4);
        stheta = theta*[omega; v];  
    end
end

end

