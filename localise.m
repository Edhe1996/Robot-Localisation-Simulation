function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.

[botSim, real_bot] = localisation_pf(botSim,map,target); %localisation by particle filter

ex_map = extend_map(3, map);
real_pos=real_bot.getBotPos();
n_nodes = size(ex_map, 1);
dist = zeros(n_nodes, 1);
for i = 1:n_nodes
    dist(i) = (ex_map(i, 1) - real_pos(1))^2 + (ex_map(i, 2) - real_pos(2))^2;
end
[~, max_index] = max(dist);
max_node = ex_map(max_index, :); %go to this vertex to check if the localisation is right
[botSim, real_bot] = pathPlan(botSim,real_bot,max_node,map); %check the localisation right or not

[botSim, ~] = pathPlan(botSim,real_bot,target,map);  %path planning by Dijkstra
end
