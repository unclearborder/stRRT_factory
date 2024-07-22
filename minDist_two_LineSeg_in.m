function [min_dist, is_cross] = minDist_two_LineSeg_in(x1_st, x1_end, x2_st, x2_end)
% 2つの線分間の最小距離を算出する関数
% [x1_st; x1_end] and [x2_st; x2_end]
% もし上記の2つの線分のが交差するとき，「min_dist = 0 && Is_cross = true」 となる．

% Two line segments
% [x1, y1, x2-x1, y2-y2]
line_seg1 = [x1_st, x1_end - x1_st]; 
line_seg2 = [x2_st, x2_end - x2_st];
is_cross = Is_two_lineseg_cross(line_seg1, line_seg2);

if is_cross == true
    min_dist = 0;
    return
else
    % 2つの線分が交差しないとき，最小距離は...
    min_dist = two_line_seg_mindist(x1_st, x1_end, x2_st, x2_end);
end

end


function min_dist = two_line_seg_mindist(x1_st, x1_end, x2_st, x2_end)

% Calculate the minimum distance from each end point to the other line segment.

% From the end points of x1 to the line segment x2
x1_st_2_LineSeg2 = dist_point2lineseg(x1_st, x2_st, x2_end);
x1_end_2_LineSeg2 = dist_point2lineseg(x1_end, x2_st, x2_end);

% From the end points of x2 to the line segment x1
x2_st_2_LineSeg1 = dist_point2lineseg(x2_st, x1_st, x1_end);
x2_end_2_LineSeg1 = dist_point2lineseg(x2_end, x1_st, x1_end);

min_dist = min([x1_st_2_LineSeg2, x1_end_2_LineSeg2, x2_st_2_LineSeg1,...
    x2_end_2_LineSeg1]);

end


% Calculate the distance between point pt and the line segment 
function dist_pt2LineSeg = dist_point2lineseg(pt, line_St, line_Ed)
    line_seg = line_Ed - line_St;
    lineSt2_pt = pt - line_St;
    
    cross_twoLine = dot(lineSt2_pt, line_seg);
    cross_lineSeg = dot(line_seg, line_seg);
    
    if cross_twoLine <= 0
        dist_pt2LineSeg = norm(lineSt2_pt);
    elseif cross_twoLine < cross_lineSeg
        d2 = cross_twoLine / norm(line_seg);
        dist_pt2LineSeg = sqrt( norm(lineSt2_pt)^2 - d2^2 );
    else
        lineEd2_pt = pt - line_Ed;
        dist_pt2LineSeg = norm(lineEd2_pt);
    end
end







