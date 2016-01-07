% Code for generating a finite difference matrix

%TODO: 
%   
function [A] = finitediff(n)
    
    A = zeros(n,n);
    
    A(1,1) = 1;
    A(n,n) = 1;
    
    A(2,1) = -2;
    A(n-1,n) = -2;
    
    A(2,2) = 1;
    A(n-1,n-1) = 1;
    
    for j = 3:(n / 2)
        A(j,j-2) = 1;
        A(j,j-1) = -2;
        A(j,j) = 1;
        
        A(n-j+1,n-j+3) = 1;
        A(n-j+1,n-j+2) = -2;
        A(n-j+1,n-j+1) = 1;
    end
    
    if mod(n, 2) == 1
        index = int8(n / 2);
        A(index,index-1) = 1;
        A(index,index) = -2;
        A(index,index+1) = 1;
    end
    
end