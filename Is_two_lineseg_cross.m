function [is_cross] = Is_two_lineseg_cross(edge1, edge2)

% checks if two line segment cross each other

is_cross = false;

% Is two line segments' starting point match?
if abs(edge2(1) - edge1(1)) <= 10^-5 && abs(edge2(2) - edge1(2)) <= 10^-5
    is_cross = true;
    return
end
den = det([edge2(3), edge1(3); edge2(4), edge1(4)]);

% Are two line segments parallel?
if den == 0
    return
end

% Are two line segments cross?
s = 1/den * ( (edge1(1) - edge2(1))*edge1(4) - (edge1(2) - edge2(2))*edge1(3) );
t = -1/den * ( -(edge1(1) - edge2(1))*edge2(4) + (edge1(2) - edge2(2))*edge2(3) );

if s >= 0 && s <= 1 && t >= 0 && t <= 1
    is_cross = true;
    return
end
end