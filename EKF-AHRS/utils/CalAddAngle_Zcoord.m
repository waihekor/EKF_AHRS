function add_angle = CalAddAngle_Zcoord( begin_angle,end_angle )
    begin_rad = begin_angle*pi/180; 
    end_rad = end_angle*pi/180 ;
    begin_matrix = [cos(begin_rad),-sin(begin_rad),0;sin(begin_rad),cos(begin_rad),0;0,0,1];
    end_xy = [cos(end_rad),sin(end_rad),0]';
    add_xy = begin_matrix'* end_xy;
    add_rad = atan2(add_xy(2), add_xy(1));
    add_angle = add_rad/pi*180;
    
end
