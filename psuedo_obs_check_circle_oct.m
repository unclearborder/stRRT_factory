function issue_flag = psuedo_obs_check_circle_oct(x, node_jj, map, Dc)
    issue_flag = false;
%     issue_boundary = true;
    x0 = node_jj.x(1:2);
    xF = x(1:2);
    obs_c = map.AreaMap.offset_center;

    d = abs(det([xF-x0;obs_c-x0]))/norm(xF-x0);
    if d < Dc
        issue_flag = true;
        return
    else
        issue_flag = false;
    end
end