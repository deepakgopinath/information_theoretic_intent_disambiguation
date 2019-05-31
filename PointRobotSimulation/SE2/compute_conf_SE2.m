function conf = compute_conf_SE2( uh, xr )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global ng xg;
if size(xr, 2) > 1
    xr = xr';
end
conf = (1/ng)*ones(ng, 1);

for i=1:ng %directedness based confidence function. 
    normxs = (xg(1:2, i) - xr(1:2))/(norm((xg(1:2, i) - xr(1:2))) + realmin); %ith goal position;
    normuh = uh(1:2)/(norm(uh(1:2)) +realmin); %add realmin to avoid divide by zero errors. 
    costh = dot(normxs, normuh);
    conf(i) = conf(i) + ((1 + costh)/2);
end
if abs(uh(3)) > 0
    for i=1:ng
            %find shortest angle to the goal. If the sign of that angle and
            %velocity is the same, give a score of 1. else 0. For the time
            %being. We are assuming the user is operating at max velocity. 
            if xr(3) < xg(3,i)
                if abs(xg(3,i) - xr(3)) <= pi
                    diff_angle = xg(3,i) - xr(3);
                else
                    diff_angle = abs(xg(3,i) - xr(3)) - 2*pi;
                end
                if sign(uh(3)) == sign(diff_angle)
                    conf(i) = conf(i) + 1;
                else
                    conf(i) = conf(i) + 0;
                end
            elseif xr(3) > xg(3, i)
                if abs(xg(3,i) - xr(3)) <= pi
                    diff_angle = xg(3,i) - xr(3);
                else
                    diff_angle = 2*pi - abs(xg(3,i) - xr(3));
                end
                if sign(uh(3)) == sign(diff_angle)
                    conf(i) = conf(i) + 1;
                else
                    conf(i) = conf(i) - 0;
                end
            else
                conf(i) = conf(i) + 0.5;
            end
    end
end
conf = conf/(sum(conf) + realmin);
end

