function plotobj = plot_robot_3D(c,Dc,color)
      num_points = 50;
      [X,Y,Z] = sphere(num_points);
      X = X*Dc+c(1);
      Y = Y*Dc+c(2);
      Z = Z*0+c(3);

      plotobj = surf(X,Y,Z,'FaceColor',color,'EdgeColor',color,'FaceAlpha',0.1,'EdgeAlpha',0.3);

end