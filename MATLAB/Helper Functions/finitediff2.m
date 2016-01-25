% Function to generate a finite difference matrix using the center
% differentiation rule

function [A] = finitediff2(n)
    
    A = zeros(n,n);
    
    A(1,1) = -2;
    A(n,n) = -2;
    
    A(1,2) = 1;
    A(n,n-1) = 1;
    
    
    for j = 2:n-1
        A(j,j-1) = 1;
        A(j,j) = -2;
        A(j,j+1) = 1;
    end    
end