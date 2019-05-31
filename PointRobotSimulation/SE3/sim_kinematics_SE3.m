function xr_new_T = sim_kinematics_SE3(xr_T, uh) %all args are considered to be column vectors. 
    global delta_t;
    xr_new_T = zeros(4,4); xr_new_T(4,4) = 1.0; %create empty homogeneous matrix. 
    xr = xr_T(1:3, 4); %get curr position
    uh_t = uh(1:3); %current translational velocity
    xr_new_T(1:3, 4) = xr + uh_t.*delta_t + normrnd(0, 0.000, length(xr), 1); %update translation
    
    %deal with rotation
    R = xr_T(1:3, 1:3); %current rotation matrix
    xr_new_T(1:3, 1:3) = MatrixExp3(uh(4:6).*delta_t)*R; %update rotation uh(4:6) angular velocity wrt world frame. 
    %apply rotation
end