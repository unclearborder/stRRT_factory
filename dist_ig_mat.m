function d_cont = dist_ig_mat( x1_mat, x2)
% x1_matノードからxノードへの距離を計算する． 
d_cont = sqrt( sum( (x2 - x1_mat ).^2) );
