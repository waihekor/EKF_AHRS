function [pitch0,roll0,yaw0] = Get_Init_AHRS(acce_data,mag_data)
    pitch0 = asin(acce_data(2)/norm(acce_data,2));
    roll0 = atan2(-acce_data(1),acce_data(3));
    
    m_x = mag_data(1)*cos(roll0)+mag_data(3)*sin(roll0);
    m_y = mag_data(1)*sin(pitch0)*sin(roll0)+ mag_data(2)*cos(pitch0)-mag_data(3)*cos(roll0)*sin(pitch0); 
    yaw0 = atan2(m_y,m_x);
end
