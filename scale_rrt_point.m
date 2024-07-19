function [x, plotobj_scale_newpoint] = scale_rrt_point(d,radius,x,node,nearest)

% Check if the node is within the predefined distance (radius), if not it scales it

if d > radius
    
    plotobj_line = plot3([x(1) node(nearest).x(1)],[x(2) node(nearest).x(2)],[x(3) node(nearest).x(3)],'k--','LineWidth',2);
    
    nearest_x = node(nearest).x;
    direction_vec = x-nearest_x;
    direction_unit_vec = direction_vec/norm(direction_vec);  

    x = nearest_x + radius*direction_unit_vec;
    
    plotobj_scale_newpoint = plot3(x(1),x(2),x(3),'r.','MarkerSize',15);
    
    delete(plotobj_line);
    
else
    plotobj_line = plot3([x(1) node(nearest).x(1)],[x(2) node(nearest).x(2)],[x(3) node(nearest).x(3)],'k--','LineWidth',2);
    plotobj_scale_newpoint = plot3(x(1),x(2),x(3),'r.','MarkerSize',15);
    delete(plotobj_line);
end
    
    
end