function [botSim, real_bot] =  pathPlan(botSim,real_bot,target,map)
%%Find the path to the target and plot it in debug mode

real_pos=real_bot.getBotPos(); 
ex_map=extend_map(3, map); % change the border to avoid collision
location_threshold=0.005; 

% botScan = botSim.ultraScan();
% num_scan = size(botScan);
% [max_dis, max_dis_ind] = max(botScan); %move along the longest distance direction
% move = rand*max_dis*0.7; 
% turn = 2*pi*(max_dis_ind-1)/num_scan;
% if botScan(1)<move
% % disp('Collision. Relocalise!');
%     [botSim, real_bot] = localisation_pf(botSim,map,target); %relocalise
%     real_pos=real_bot.getBotPos();
% else
%     botSim.turn(turn);        
%     botSim.move(move); 
%     real_bot.turn(turn);
%     real_bot.move(move);
% end
%% Path plan and get to the target
while real_pos(1) > target(1) + location_threshold || real_pos(1) < target(1) - location_threshold || real_pos(2) > target(2) + location_threshold|| real_pos(2) < target(2) - location_threshold
     
    shortest_path = path_Dijkstra(real_pos, target, ex_map); %find the path to the target, based on Dijkstra's algorithm
    
    for i=1:size(shortest_path,1)-1 %go to the target step by step
        
        start_node=shortest_path(i,:);
        end_node=shortest_path(i+1,:);
    
        turn_angle =(atan2d(end_node(2)-start_node(2),end_node(1)-start_node(1))+180)*pi/180; %angle between start and end nodes
    
        move_distance= distance(start_node,end_node); %distance between start and end nodes
    
        botSim.turn(pi-real_bot.getBotAng()+turn_angle); 
        real_bot.turn(pi-real_bot.getBotAng()+turn_angle);
    
        %make sure do not crash on the walls for every move
        botScan=botSim.ultraScan();
        if botScan(1)<move_distance
            %disp('Collision. Relocalise!');
            [botSim, real_bot] = localisation_pf(botSim,map,target); %relocalise
            real_pos=real_bot.getBotPos();  
            break;
        else
           if botSim.debug()                
                hold off; %the drawMap() function will clear the drawing when hold is off        
                botSim.drawMap(); %draw the real bot
                botSim.drawBot(30,'g'); %draw robot with line length 30 and green
                real_bot.drawMap(); %draw the estimated real bot
                real_bot.drawBot(5,'b'); %draw robot with line length 5 and blue
                plot(target(1),target(2),'*'); %draw the target
                ex_plot=ex_map;
                ex_plot(size(ex_map,1)+1,:)=ex_map(1,:) ;   
                plot(ex_plot(:,1), ex_plot(:,2), 'Color', 'black'); %draw the extended map
                plot(shortest_path(:,1), shortest_path(:,2),'-ko', 'Color', 'blue');   
           end            
            botSim.move(move_distance);
            real_bot.move(move_distance); 
            real_pos=real_bot.getBotPos();  
        end
    end
end  
if botSim.debug()
    hold off; %the drawMap() function will clear the drawing when hold is off        
    botSim.drawMap(); %drawMap() turns hold back on again
    botSim.drawBot(30,'g'); %draw robot with line length 30 and green
    real_bot.drawMap(); %drawMap() turns hold back on again, so you can draw the estimated real bots
    real_bot.drawBot(5,'r'); %draw robot with line length 5 and red
    plot(target(1),target(2),'*');
    ex_plot=ex_map;
    ex_plot(size(ex_map,1)+1,:)=ex_map(1,:) ;   
    plot(ex_plot(:,1), ex_plot(:,2), 'Color', 'black');
    plot(shortest_path(:,1), shortest_path(:,2), '-ko', 'Color', 'blue');
end
end