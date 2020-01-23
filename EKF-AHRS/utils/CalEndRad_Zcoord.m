function end_rad = CalEndRad_Zcoord( begin_rad,add_rad )

    add_matrix = [cos(add_rad),-sin(add_rad),0;sin(add_rad),cos(add_rad),0;0,0,1];
    begin_xy = [cos(begin_rad),sin(begin_rad),0]';
    end_xy = add_matrix*begin_xy;
    end_rad = atan2(end_xy(2),end_xy(1));

end






