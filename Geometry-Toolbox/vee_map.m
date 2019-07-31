function t = vee_map(A, force)
    if(nargin == 1)
        force = false ;
    end
    if(~force)
        if(double(sum(abs(diag(A)))) > 1e-2)
            disp('Error in vee_map() - Nonzero diagonal') ;
%             keyboard ;
        end
    end
    %A = [0 -t(3) t(2) ; t(3) 0 -t(1) ; -t(2) t(1) 0] ;
    t = [A(3,2) ;
         A(1,3) ;
         A(2,1)] ;
end