function issue_flag = psuedo_obs_check_circle_oct(x, node_jj, obs_c, obs_r, Dc)
%     issue_boundary = true;
    x0 = node_jj.x(1:2);
    xF = x(1:2);
    d = abs(det([xF-x0;obs_c-x0]))/norm(xF-x0);
    if d <= Dc + obs_r
        issue_flag = true;
        return
    elseif d > Dc + obs_r
        issue_flag = false;
    end
end