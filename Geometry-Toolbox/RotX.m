function R = RotX(th)
    R = [1 0 0 ;
        0 cos(th) -sin(th) ;
        0 sin(th)  cos(th)] ;
end