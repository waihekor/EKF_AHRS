function [pitch,roll,yaw] = quattoeuler(q)
    rad2deg=180/pi;
    R_n2b=[ 1 - 2 * (q(4) *q(4) + q(3) * q(3)) 2 * (q(2) * q(3)-q(1) * q(4))         2 * (q(2) * q(4) +q(1) * q(3)) ;
        2 * (q(2) * q(3) +q(1) * q(4))     1 - 2 * (q(4) *q(4) + q(2) * q(2))    2 * (q(3) * q(4)-q(1) * q(2));
        2 * (q(2) * q(4)-q(1) * q(3))      2 * (q(3) * q(4)+q(1) * q(2))         1 - 2 * (q(2) *q(2) + q(3) * q(3))];%cnb

    pitch = asin(R_n2b(3,2))*rad2deg;
    roll  = atan2(-R_n2b(3,1),R_n2b(3,3))*rad2deg;
    yaw   = atan2(-R_n2b(1,2),R_n2b(2,2))*rad2deg;  
end


