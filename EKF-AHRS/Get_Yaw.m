function [yaw0] = Get_Yaw(pitch_angle,roll_angle,mag_data)    

    pitch = pitch_angle*pi/180;
    roll = roll_angle*pi/180;   
    m_x = mag_data(1)*cos(roll)+mag_data(3)*sin(roll);
    m_y = mag_data(1)*sin(pitch)*sin(roll)+ mag_data(2)*cos(pitch)-mag_data(3)*cos(roll)*sin(pitch); 
    yaw0 = atan2(m_y,m_x)*180/pi;
end
