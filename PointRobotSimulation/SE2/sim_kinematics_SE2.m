function xr_new = sim_kinematics_SE2(xr, uh) %all args are considered to be column vectors. 
    global delta_t;
    xr_new = xr + uh.*delta_t + normrnd(0, 0.000, length(xr), 1); %add stochasticity to the kinematics
    %wrap 3rd element to 0 to 2pi
    xr_new(3) = wrapTo2Pi(xr_new(3)); 
end