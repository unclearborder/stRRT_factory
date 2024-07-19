clear all
close all

%% map read
[str,dirname,ext] = fileparts(pwd);
% addpath(append(str,'\function'))

map_list = {'Area1'};
map_limit = [45000,30000];
Area_Number = length(map_list);
ReadFile_map = append('C:\Users\RobRoy\Documents\MATLAB\TaskScheduling_Excel\Area_axisdata.xlsx');
ReadFile_Target = append('C:\Users\RobRoy\Documents\MATLAB\TaskScheduling_Excel\Target_axisdata.xlsx');
map_data = cell(Area_Number,1);
Target_data = cell(Area_Number,1);

for r = 1:Area_Number
    map_data{r}.data = readtable(ReadFile_map,'Sheet',map_list{r});
    map_data{r}.map_offset = [-max(map_data{r}.data.x)/2,max(map_data{r}.data.y)];
    map_data{r}.objectNum = size(map_data{r}.data.name,1);

    % wall setting
    map_data{r}.data.offset_start = zeros(map_data{r}.objectNum,2);
    map_data{r}.data.offset_finish = zeros(map_data{r}.objectNum,2);
    map_data{r}.data.offset_center = zeros(map_data{r}.objectNum,2);

    for i = 1:map_data{r}.objectNum
        % wall_setting
        if ~strcmp(map_data{r}.data.type(i),'circle') == true
            if strcmp(map_data{r}.data.offset_base_x(i),'left') == true
                map_data{r}.data.offset_start(i,1) = map_data{r}.map_offset(1)+map_data{r}.data.offset_x(i);
                map_data{r}.data.offset_finish(i,1) = map_data{r}.data.offset_start(i,1) + map_data{r}.data.x(i);
            else
                [map_data{r}.data.offset_start(i,1),map_data{r}.data.offset_finish(i,1)] = offset_rectangle_x(map_data{r}.data,i);
            end

            if strcmp(map_data{r}.data.offset_base_y(i),'north') == true
                map_data{r}.data.offset_start(i,2) = map_data{r}.map_offset(2)-map_data{r}.data.offset_y(i);
                map_data{r}.data.offset_finish(i,2) = map_data{r}.data.offset_start(i,2) - map_data{r}.data.y(i);
            else
                [map_data{r}.data.offset_start(i,2),map_data{r}.data.offset_finish(i,2)] = offset_rectangle_y(map_data{r}.data,i);
            end
        elseif strcmp(map_data{r}.data.type(i),'circle') == true
            if strcmp(map_data{r}.data.offset_base_x(i),'left') == true
                map_data{r}.data.offset_center(i,1) = map_data{r}.map_offset(1)+map_data{r}.data.offset_x(i);
            else
                map_data{r}.data.offset_center(i,1) = offset_circle_x(map_data{r}.data,i);
            end
            
            if strcmp(map_data{r}.data.offset_base_y(i),'north') == true
                map_data{r}.data.offset_center(i,2) = map_data{r}.map_offset(2)-map_data{r}.data.offset_y(i);
            else
                map_data{r}.data.offset_center(i,2) = offset_circle_y(map_data{r}.data,i);
            end

        end
    end

    Target_data{r}.data = readtable(ReadFile_Target,'Sheet',map_list{r});
    Target_data{r}.Sum_of_Target = length(Target_data{r}.data.TargetNum);
    Target_data{r}.data.axis(1,:) = [map_data{r}.map_offset(1)+Target_data{r}.data.path_x(1);
                                     map_data{r}.map_offset(2)-Target_data{r}.data.path_y(1)]';
    for j = 2:Target_data{r}.Sum_of_Target
        near_j = Target_data{r}.data.near(j);
        Target_data{r}.data.axis(j,:) = [Target_data{r}.data.axis(near_j,1)+Target_data{r}.data.path_x(j);
                                         Target_data{r}.data.axis(near_j,2)-Target_data{r}.data.path_y(j)]';
    end


end

save map_data.mat
save Target_data.mat