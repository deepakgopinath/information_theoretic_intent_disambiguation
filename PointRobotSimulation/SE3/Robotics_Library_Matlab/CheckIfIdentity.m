function [ isidentity ] = CheckIfIdentity( X )

%Utility function to check whether a quantity M is a valid identity matrix. 
%Input - M - scalar, matrix, vector....
%Output - True - if valid identity, False if not identity

dim = size(X, 1); %already assumes this is square
epsilon = 10^-5;
isidentity = true;
for i=1:dim
    if(isidentity)
        for j=1:dim
            if( i==j)
                if(abs(X(i,j) - 1) > epsilon)
                    isidentity = false;
                    break;
                end
            else
                if(abs(X(i,j) - 0) > epsilon)
                    isidentity = false;
                    break;
                end
            end
        end
    else
        break;
    end
end
end

