function path_coordinates = path_Dijkstra(start_point, end_point, extend_map)
%%find the path from start_point to end_point in extend_map
%%initialise all nodes of the graph
ini_nodes = zeros(size(extend_map,1)+2,3); %3 columns for all vertice plus start and end points
ini_nodes(1,1:2) = [start_point(1), start_point(2)]; %first and second lolumn are x and y, start node (current)
ini_nodes(2:size(extend_map,1)+1,1:2) = extend_map; %map nodes
ini_nodes(size(extend_map,1)+2,1:2) = [end_point(1), end_point(2)]; %target node
ini_nodes(:,3) = Inf*ones(size(ini_nodes,1),1); %third column is cost, Inf stands for unvisited

visitible_nodes_ID = zeros(1,size(ini_nodes,1)); %visibility

nb_lib =  zeros(size(ini_nodes,1),size(ini_nodes,2),size(ini_nodes,1));

visitible_id = 0;

for all_nodes = 1:size(ini_nodes,1)
    combined_nodes = ini_nodes;
    
    for current_neighbor_nodes = 1:size(ini_nodes,1) 
        current_node = ini_nodes(all_nodes,:);
        current_neighbors = ini_nodes(current_neighbor_nodes,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        %Check visitibility of neighbors from current node
        x_current = current_node(1);
        y_current = current_node(2);
        x_current_neighbor = current_neighbors(1);
        y_current_neighbor = current_neighbors(2);
 
        distance_array = zeros(1,size(extend_map,1)); %distance from current node to its neighbors
        i = 0;

        for k = 1:1:size(extend_map,1) % for all the wall, check if it on the path of current node and its neighbors
            if k < size(extend_map,1) %the wall
                point_1 = [extend_map(k,1);extend_map(k,2)];
                x_1 = point_1(1);
                y_1 = point_1(2);
                point_2 = [extend_map(k+1,1);extend_map(k+1,2)];
                x_2 = point_2(1);
                y_2 = point_2(2);
            else
                point_1 = [extend_map(k,1);extend_map(k,2)];
                x_1 = point_1(1);
                y_1 = point_1(2);
                point_2 = [extend_map(1,1);extend_map(1,2)];
                x_2 = point_2(1);
                y_2 = point_2(2);
            end
 
            path_direction = [(x_current_neighbor - x_current) ; (y_current_neighbor - y_current)]; %direction of path
            wall_direction = [(x_2-x_1); (y_2-y_1)]; %direction of current wall
            %Check whether the two lines have intersections or not
            intersection_check = (dot(path_direction,wall_direction))/( norm(path_direction)*norm(wall_direction));
            if intersection_check ~= 1 && intersection_check ~= -1 %Make sure that the lines are not parallel (and therefore will intersect)

                %calculate the distance to wall expressed as a multiplier to the direct distance between the current node and curent neighbor
                numerator = (x_2 - x_1)*(y_1 - y_current) - (y_2 - y_1)*(x_1 - x_current);
                denominator = (x_2 - x_1)*(y_current_neighbor - y_current) - (y_2 - y_1)*(x_current_neighbor - x_current);
                p = numerator/denominator;

                %Check if particle is actually facing wall (true if p >= 0)
                if p >= 0
                    if (y_2 - y_1) == 0 %whether wall is horizontal or not
                        %calculate intersection position
                        q = ( x_current - x_1 + p*(x_current_neighbor - x_current) )/(x_2 - x_1);
                    elseif (x_2 - x_1) == 0 %vertical
                        q = ( y_current - y_1 + p*(y_current_neighbor - y_current ) )/(y_2 - y_1);
                    else %neither vertical nor horizontal
                        q = ( y_current - y_1 + p*(y_current_neighbor - y_current ) )/(y_2 - y_1);
                    end
                    %Check if intersection happened within the wall ends
                    if q >= 0 && q <= 1
                        i = i + 1;
                        distance_array(i) = p;
                    end
                end
            end
        end

        p_min = min(distance_array(1:i));
        if p_min == 0 %current node on a wall
            nonzero_entry_index = (distance_array(1:i) ~= 0); %Only consider nonzero entries i.e. walls that are not coincident with the current node
            p_min = min(distance_array(nonzero_entry_index)); %overwrite p_min
        end

        if p_min < 1 %If wall encounter between current node and current neighbor
            visit_status = 0; %definitely not a neighbor
        else
            x_midpoint = 0.5*(x_current + x_current_neighbor); %middle point of the current node and current neighbor
            y_midpoint = 0.5*(y_current + y_current_neighbor);

            [IN, ON] = inpolygon(x_midpoint,y_midpoint,extend_map(:,1),extend_map(:,2)); % mid_point in the map or on the map or not

            if IN == 1 || ON == 1
                visit_status = 1; %a neighbor
            else
                visit_status = 0; %not a neighbor
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        if visit_status == 1
            visitible_id = visitible_id + 1;
            visitible_nodes_ID(visitible_id) = current_neighbor_nodes;
            %the distance from current node to visitible nodes(neighbors)
            combined_nodes(current_neighbor_nodes,3) = sqrt((current_neighbors(1) - current_node(1))^2 + (current_neighbors(2) - current_node(2))^2);             
        end   
    end
    
    nb_lib(:,:,all_nodes) = combined_nodes;
end
%Now we have a graph. Then we need to perform the Dijkstra algorithm
open_list = zeros(1,size(ini_nodes,1)); %initialize unvisited nodes (open list)

%generate a list of all available nodes and assign them to the open list
for index = 1:size(ini_nodes)
    open_list(index) = index;
end

shortest_path_array = nb_lib(:,:,1); %the shortest path
shortest_path_array(:,4) = 1;

current_node = 1;
open_list = setdiff(open_list, current_node); %update the open list

while size(open_list,2) > 0 %iterate over all nodes in the open list
    cumulative_distances = shortest_path_array(open_list,3);
    [~, current_node_index] = min(cumulative_distances);
    
    current_node_ID = open_list(current_node_index);
    open_list = setdiff(open_list, current_node_ID); %update the open list

    for unvisited_node_index = 1:size(open_list,2)
        neighbor_ID = open_list(unvisited_node_index);

        if nb_lib(neighbor_ID,3,current_node_ID) < Inf
            distance_c = shortest_path_array(current_node_ID,3);
            distance_c2n = nb_lib(neighbor_ID,3,current_node_ID);
            new_cumulative_distance = distance_c + distance_c2n;
            pre_cumulative_distance = shortest_path_array(neighbor_ID,3);
            if new_cumulative_distance < pre_cumulative_distance %if the current path is shorter
                shortest_path_array(neighbor_ID,3) = new_cumulative_distance; %update the path to this target neighbor
                shortest_path_array(neighbor_ID,4) = current_node_ID;                
            end         
        end     
    end 
end

%get the shortest path
path = zeros(1,size(shortest_path_array,1));
path_index = 1; %the start point
path(path_index) = size(ini_nodes,1);

while path(path_index) > 1 %iterate until the start point is reached  
    path_index = path_index + 1;
    path(path_index) = shortest_path_array(path(path_index-1),4);  
end

path = fliplr(path(1:path_index));

path_coordinates(:,1) = ini_nodes(path,1); %the coordinates of all points on the shortest path
path_coordinates(:,2) = ini_nodes(path,2);
end