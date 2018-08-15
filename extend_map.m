function [ex_Map] = extend_map( offset_dist, map)
%reference: http://stackoverflow.com/questions/23543212/defining-an-inside-room-point-from-wall-points/23548238#23548238

% initialise
number = size(map,1); %numbers of vertices
vertex1 = zeros(number, 2);
vertex2 = zeros(number, 2);
unit_vector = zeros(number, 2); %unit vector
edge = zeros(number, 1); 
angle = zeros(number, 1);

for i = 1:number      
    if i == number
        edge(i) = (map(1,1) - map(i,1))*(map(1,2) + map(i,2));
    else
        edge(i) = (map(i+1,1) - map(i,1))*(map(i+1,2) + map(i,2));
    end
end

%determine the orientation of vertices
if sum(edge) > 0 %anti-clockwise
    orienation = -1;
elseif sum(edge) < 0 %clockwise
    orienation = 1;
end

for i = 1:number
    curr = map(i,:); %current point
    if i == 1
        prev = map(number, :);
    else
        prev = map(i-1,:);
    end
    if i == number
        next = map(1, :);
    else
        next = map(i+1, :);
    end

    vertex1(i,:) = next - curr; 
    vertex2(i,:) = prev - curr;

    unit_vector(i,:) = vertex1(i,:)/norm(vertex1(i,:)); %unit vector of v1
    x1 = vertex1(i,1);
    y1 = vertex1(i,2);
    x2 = vertex2(i,1);
    y2 = vertex2(i,2);
    angle(i) = mod(atan2(x1*y2-x2*y1,x1*x2+y1*y2),2*pi)*180/pi;
end

newAngle = angle./2; 
new_vertices = zeros(number, 2);
ex_Map = zeros(number, 2); %initialize extended map vertices
offset = zeros(number, 1); %offset for each vertex

% find new vertices
for i = 1:number
    if angle(i) <= 180
        offset(i) = orienation * offset_dist/sind(newAngle(i));
        rotation = [cosd(newAngle(i)), sind(newAngle(i)); -sind(newAngle(i)), cosd(newAngle(i))];
        new_vertices(i,:) = unit_vector(i,:) * rotation * offset(i);
        ex_Map(i,:) = map(i,:) + new_vertices(i,:);
    elseif angle(i) > 180 && angle(i) <= 270
        offset(i) = -orienation * offset_dist/sind(newAngle(i)-180);
        rotation = [cosd(newAngle(i)), sind(newAngle(i)); -sind(newAngle(i)), cosd(newAngle(i))];
        new_vertices(i,:) = unit_vector(i,:) * rotation * offset(i);
        ex_Map(i,:) = map(i,:) + new_vertices(i,:);
    elseif angle(i) > 270
        offset(i) = -orienation * offset_dist/sind(newAngle(i)-180);
        rotation = [cosd(newAngle(i)), sind(newAngle(i)); -sind(newAngle(i)), cosd(newAngle(i))];
        new_vertices(i,:) = unit_vector(i,:) * rotation * offset(i);
        ex_Map(i,:) = map(i,:) + new_vertices(i,:);
    end
end
end

