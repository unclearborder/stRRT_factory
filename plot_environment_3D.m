
load get_mapinfo/map_data
load get_mapinfo/Target_data

% t_max = 2400;

figure;
hold on
grid on
fig_f = gcf;
fig_f.Position = [-1067 -96 1059 681];

    
plotlimX= [map_data{r}.map_offset(1) map_data{r}.map_offset(1)+map_limit(1)];
plotlimY = [map_data{r}.map_offset(2)-map_limit(2) map_data{r}.map_offset(2)];
plotlimZ = [0 t_max];
axis([1.1*plotlimX 1.1*plotlimY plotlimZ])
xlabel("Location X [m]")
ylabel("Location Y [m]")
zlabel("Prediction Time[s]")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 22)

view(3)

for i = 1:map_data{r}.objectNum
        if strcmp(map_data{r}.data.type(i),'circle') == true
            [obj_x0,obj_y0,obj_z0] = cylinder(map_data{r}.data.r(i));
            obj_z = obj_z0*t_max;
            obj_x = obj_x0 + map_data{r}.data.offset_center(i,1);
            obj_y = obj_y0 + map_data{r}.data.offset_center(i,2);
            surf(obj_x,obj_y,obj_z,'FaceAlpha',0.02,'EdgeColor','g','LineWidth',3,'EdgeAlpha',0.2);
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
            patch('Vertices',obj_v_base,'Faces',obj_f,'FaceColor','g','FaceAlpha',0.02);
            patch('Vertices',obj_v_tlim,'Faces',obj_f,'FaceColor','g','FaceAlpha',0.02);

            SideIndex = [1 5 6 2;
                         4 8 7 3;
                         1 5 8 4;
                         2 6 7 3];
            obj_v_t = [obj_v_base; obj_v_tlim];
            for j = 1:length(SideIndex)
                patch('Vertices',obj_v_t(SideIndex(j,:),:),'Faces',obj_f,'FaceColor','g','FaceAlpha',0.02);
            end
        else
            obj_f = [1 2 3 4];
            obj_v = [map_data{r}.data.offset_start(i,1) map_data{r}.data.offset_start(i,2);
                     map_data{r}.data.offset_finish(i,1) map_data{r}.data.offset_start(i,2);
                     map_data{r}.data.offset_finish(i,1) map_data{r}.data.offset_finish(i,2);
                     map_data{r}.data.offset_start(i,1) map_data{r}.data.offset_finish(i,2)];
            obj_v_base = [obj_v zeros(4,1)];
            obj_v_tlim = [obj_v t_max*ones(4,1)];
            patch('Vertices',obj_v_base,'Faces',obj_f,'FaceColor','g','FaceAlpha',0.2);
            patch('Vertices',obj_v_tlim,'Faces',obj_f,'FaceColor','g','FaceAlpha',0.2);

            SideIndex = [1 5 6 2;
                         4 8 7 3;
                         1 5 8 4;
                         2 6 7 3];
            obj_v_t = [obj_v_base; obj_v_tlim];
            for j = 1:length(SideIndex)
                patch('Vertices',obj_v_t(SideIndex(j,:),:),'Faces',obj_f,'FaceColor','g','FaceAlpha',0.2);
            end
        end
end
% plot(Target_data{r}.data.axis(:,1),Target_data{r}.data.axis(:,2),'k.','MarkerSize',20)
plot(Target_data{r}.data.axis(Targetst,1),Target_data{r}.data.axis(Targetst,2),'r.','MarkerSize',20)
tgt_v = [target([1;1;2;2],1),target([1;2;2;1],2)];
tgt_f = [1 2 3 4];
tgt_v_base = [tgt_v zeros(4,1)];
tgt_v_tlim = [tgt_v t_max*ones(4,1)];
patch('Vertices',tgt_v_base,'Faces',tgt_f,'FaceColor','m','FaceAlpha',0.2);
patch('Vertices',tgt_v_tlim,'Faces',tgt_f,'FaceColor','m','FaceAlpha',0.2);
SideIndex = [1 5 6 2;
             4 8 7 3;
             1 5 8 4;
             2 6 7 3];
tgt_v_t = [tgt_v_base; tgt_v_tlim];
for j = 1:length(SideIndex)
    patch('Vertices',tgt_v_t(SideIndex(j,:),:),'Faces',tgt_f,'FaceColor','m','FaceAlpha',0.2);
end

%% Figure of search area considering max velocity of robot
% t = 0:10:t_max;
% l = v_max*t;
% [X,Y,Z] = cylinder(l);
% X = X + node(1).x(1)*ones(length(X),1);
% Y = Y + node(1).x(2)*ones(length(Y),1);
% Z = Z*t_max;
% 
% surf(X,Y,Z,'FaceAlpha',0.5);


% 真上から見たいとき．
caz = 0.2682;
cel = 90;

% 真横から見たいとき
% caz = -6.8811;
% cel = 2.4836;

view(caz,cel)