% Code for generating a finite difference matrix that is wrong - Used for
% testing purposes

function [A] = finitediffwrong(n,x)
    
    A = zeros(n,n);
    
    A(1,1) = 1;
    A(n,n) = 1;
    
    A(2,1) = -x;
    A(n-1,n) = -x;
    
    A(2,2) = 1;
    A(n-1,n-1) = 1;
    
    for j = 3:(n / 2)
        A(j,j-2) = 1;
        A(j,j-1) = -x;
        A(j,j) = 1;
        
        A(n-j+1,n-j+3) = 1;
        A(n-j+1,n-j+2) = -x;
        A(n-j+1,n-j+1) = 1;
    end

end