function conf = compute_conf_SE3( uh, xr_T )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global ng xg_T;
xr_pos = xr_T(1:3, 4); %robot position
conf = (1/ng)*ones(ng, 1); %initialize
if norm(uh(1:3)) > 0.000001 % if translation uh is nonzero. directedness
    for i=1:ng
        curr_gT = xg_T(:,:,i);
        xg_pos = curr_gT(1:3, 4);
        normxs = (xg_pos - xr_pos)/(norm(xg_pos - xr_pos) + realmin);
        normuh = uh(1:3)/(norm(uh(1:3)) + realmin);
        costh = dot(normxs, normuh);
        conf(i) = (1 + costh)/2;
    end
end
if norm(uh(4:6)) > 0.000001 %rotational agreement. 
    for i=1:ng
        goal_vel = generate_R_autonomy(xg_T(:,:,i), xr_T);
        if norm(goal_vel(4:6)) > 0.0001
            rot_vel = goal_vel(4:6);
            rot_align = dot(uh(4:6), rot_vel)/(norm(uh(4:6)) * norm(rot_vel) +  realmin);
            rot_align = (1 + rot_align)*0.5;
            conf(i) = conf(i) + rot_align;
        end
    end
end
conf = conf/sum(conf);
end

function ur = generate_R_autonomy(gT, rT)
    ur = zeros(6,1);
    Rg = gT(1:3,1:3);  Rr = rT(1:3, 1:3);
    Rdiff = Rg*(Rr^-1); %with respect world frame, amount to turn toward goal. 
    ur(4:6) = MatrixLog3(Rdiff); %unnormalized
    [w, ~] = AxisAng3(ur(4:6)); % %normalize rotational velocity. takes care of 0 velocities internally. 
    ur(4:6) = w; %1 rad/s along w axis. 
end
