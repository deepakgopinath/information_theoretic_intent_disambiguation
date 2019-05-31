function [ normalizedVector ] = Normalize( inputVector)

%Input - a vector to be normalized. 
%Output - normalized input vector

normalizedVector = inputVector/norm(inputVector);
end

