
load get_mapinfo/map_data
load get_mapinfo/Target_data

t_max = 2400;

figure;
hold on
grid on
    
plotlimX= [map_data{r}.map_offset(1) map_data{r}.map_offset(1)+map_limit(1)];
plotlimY = [map_data{r}.map_offset(2)-map_limit(2) map_data{r}.map_offset(2)];
plotlimZ = [0 t_max];
axis([1.1*plotlimX 1.1*plotlimY plotlimZ])
xlabel("Location X [m]")
ylabel("Location Y [m]")
zlabel("Prediction Time[s]")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 22)

for i = 1:map_data{r}.objectNum
        if strcmp(map_data{r}.data.type(i),'circle') == true
            [obj_x0,obj_y0,obj_z0] = cylinder(map_data{r}.data.r(i));
            obj_z = obj_z0*t_max;
            obj_x = obj_x0 + map_data{r}.data.offset_center(i,1);
            obj_y = obj_y0 + map_data{r}.data.offset_center(i,2);
            surf(obj_x,obj_y,obj_z,'FaceAlpha',0.2);
        elseif strcmp(map_data{r}.data.type(i),'door') == true
            obj_f = [1 2 3 4];
            obj_v = [map_data{r}.data.offset_start(i,1) map_data{r}.data.offset_start(i,2);
                     map_data{r}.data.offset_finish(i,1) map_data{r}.data.offset_start(i,2);
                     map_data{r}.data.offset_finish(i,1) map_data{r}.data.offset_finish(i,2);
                     map_data{r}.data.offset_start(i,1) map_data{r}.data.offset_finish(i,2)];
            obj_v_base = [obj_v zeros(4,1)];
            obj_v_tlim = [obj_v t_max*ones(4,1)];
            patch('Vertices',obj_v_base,'Faces',obj_f,'FaceColor','black','FaceAlpha',0.1,'EdgeColor','black','LineWidth',3);
            patch('Vertices',obj_v_tlim,'Faces',obj_f,'FaceColor','black','FaceAlpha',0.1,'EdgeColor','black','LineWidth',3);

            SideIndex = [1 5 6 2;
                         4 8 7 3;
                         1 5 8 4;
                         2 6 7 3];
            obj_v_t = [obj_v_base; obj_v_tlim];
            for j = 1:length(SideIndex)
                patch('Vertices',obj_v_t(SideIndex(j,:),:),'Faces',obj_f,'FaceColor','black','FaceAlpha',0.1,'EdgeColor','black','LineWidth',3);
            end
        elseif strcmp(map_data{r}.data.type(i),'wall') == true
            obj_f = [1 2 3 4];
            obj_v = [map_data{r}.data.offset_start(i,1) map_data{r}.data.offset_start(i,2);
                     map_data{r}.data.offset_finish(i,1) map_data{r}.data.offset_start(i,2);
                     map_data{r}.data.offset_finish(i,1) map_data{r}.data.offset_finish(i,2);
                     map_data{r}.data.offset_start(i,1) map_data{r}.data.offset_finish(i,2)];
            obj_v_base = [obj_v zeros(4,1)];
            obj_v_tlim = [obj_v t_max*ones(4,1)];
            patch('Vertices',obj_v_base,'Faces',obj_f,'FaceColor','black','FaceAlpha',0.02);
            patch('Vertices',obj_v_tlim,'Faces',obj_f,'FaceColor','black','FaceAlpha',0.02);

            SideIndex = [1 5 6 2;
                         4 8 7 3;
                         1 5 8 4;
                         2 6 7 3];
            obj_v_t = [obj_v_base; obj_v_tlim];
            for j = 1:length(SideIndex)
                patch('Vertices',obj_v_t(SideIndex(j,:),:),'Faces',obj_f,'FaceColor','black','FaceAlpha',0.02);
            end
        else
            obj_f = [1 2 3 4];
            obj_v = [map_data{r}.data.offset_start(i,1) map_data{r}.data.offset_start(i,2);
                     map_data{r}.data.offset_finish(i,1) map_data{r}.data.offset_start(i,2);
                     map_data{r}.data.offset_finish(i,1) map_data{r}.data.offset_finish(i,2);
                     map_data{r}.data.offset_start(i,1) map_data{r}.data.offset_finish(i,2)];
            obj_v_base = [obj_v zeros(4,1)];
            obj_v_tlim = [obj_v t_max*ones(4,1)];
            patch('Vertices',obj_v_base,'Faces',obj_f,'FaceColor','blue','FaceAlpha',0.2);
            patch('Vertices',obj_v_tlim,'Faces',obj_f,'FaceColor','blue','FaceAlpha',0.2);

            SideIndex = [1 5 6 2;
                         4 8 7 3;
                         1 5 8 4;
                         2 6 7 3];
            obj_v_t = [obj_v_base; obj_v_tlim];
            for j = 1:length(SideIndex)
                patch('Vertices',obj_v_t(SideIndex(j,:),:),'Faces',obj_f,'FaceColor','blue','FaceAlpha',0.2);
            end
        end
end
view(3)

% for i = 