function [botSim, real_bot] = localisation_pf(botSim,map,~)
%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
% sigma = 3;   %need to decide how to set
% dampling = 0;
num_scan = 20; %number of scanning orientations
motion_noise = 0.01; %noise of moving
turn_noise = 0.005;  %noise of turning
conv_threshold=2.5; %convergence threshold

% botSim.setSensorNoise(sigma); %the sigma of Gaussian distribution
botSim.setScanConfig(botSim.generateScanConfig(num_scan));

%generate some random particles inside the map
num =400; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
prob=zeros(num,1); %probability (weight) of every particle
dist=zeros(num_scan,1); %distance (error) between real robot ant particles
%value = zeros(num_scan, 1);
%p_new(num,1) = BotSim; %new particles for resampling
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
%     particles(i).setSensorNoise(sigma);
    particles(i).setScanConfig(particles(i).generateScanConfig(num_scan));
    particles(i).setMotionNoise(motion_noise);
    particles(i).setTurningNoise(turn_noise);
end

%% Localisation code
maxNumOfIterations = 50; %max iteration time
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
            
    n = n+1;%increment the current number of iterations
    %botSim.setScanConfig(botSim.generateScanConfig(6));
    botScan = botSim.ultraScan();%get a scan from the real robot.
    
   %% Write code for updating your particles scans
   %% Write code for scoring your particles 
   for i =1:num
       particleScan = particles(i).ultraScan();
%        delta = sum( abs(particleScan - botScan) );
%        prob(i) = exp( - delta / (2 * sigma.^2) ) + dampling;
       for j=1:num_scan
               dist(j) = norm(botScan-particleScan); %difference between sensor data of real robot and particles
               particleScan = circshift(particleScan, -1);
%                dist(j) = botScan(j) - particleScan(j);
%                value(j) = exp( - (dist(j) ^ 2) / (2 * sigma^2) );
%                prob(i) = prob(i) * value(j);
       end
       [min_dis, min_pos]=min(dist);
        prob(i)= 1/ min_dis;     
        particle_turn=((min_pos-1)*2*pi)/num_scan;
        particles(i).turn(particle_turn);
   end
%    addition = 0.5 / num;
   prob = prob / sum(prob);
%    prob =  prob/sum(prob) + dampling;
   
    
    
    %% Write code for resampling your particles
    mW=max(prob);
    beta=0;
    index=randi([1, num - 1]);
    for i=1:num
        beta=beta+rand(1)*2*mW;
        while(beta>prob(index))
            beta=beta-prob(index);
            index=mod((index+1),num)+1;
            prob(i)=prob(index);
            particles(i).setBotPos(particles(index).getBotPos());
            particles(i).setBotAng(particles(index).getBotAng());
        end
    end
    for i=1:num
        particle_pos(i,:) = particles(i).getBotPos();
        particle_ang(i) = particles(i).getBotAng();
    end 
 %%estimate where the real robot is from particles
    real_bot = BotSim(modifiedMap);
    real_bot.setScanConfig(real_bot.generateScanConfig(num_scan));
    real_bot.setBotPos(mean(particle_pos));
    real_bot.setBotAng(meanangle(particle_ang));

    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        real_bot.drawBot(30,'b'); %draw extimated real bot
%         for i =1:num
%            particles(i).drawBot(3);
%         end
        drawnow;
    end
    
    %% Write code to check for convergence  
    dev=std(particle_pos); 
    
    if n > 12
        if dev <conv_threshold
            %disp('Converged!');
            converged = 1;
            threshold = 80;
            scan = real_bot.ultraScan();
            delta = sum( abs(scan - botScan) );
            if delta > threshold %precision not enough, relocalise
                %disp('Precision not enough, Relocalise!');
                converged = 0;
                n = 0;
                for i = 1:num
                    particles(i).randomPose(0);
                end
                continue
            end
            break
        end
    end
    
    if n > 45
        for i = 1:num
            particles(i).randomPose(0); %relocalise
        end
        n = 0;
    end

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    percent=0.04;
    for i=1:percent*num
       particles(randi(num)).randomPose(0);
    end
    
    
    %% Write code to decide how to move next
    if rand >0.7
        ind=randi(num_scan); %choose a random scan and move along it
        move= botScan(ind)*0.3;
        turn = 2*pi*(ind-1)/num_scan;
    else 
        [max_dis, max_dis_ind] = max(botScan); %move along the longest distance direction
        move = rand*max_dis*0.3; 
        turn = 2*pi*(max_dis_ind-1)/num_scan;   
    end
    %move the real robot
    botSim.turn(turn);        
    botSim.move(move); 
    %move the particles
    for i =1:num 
          particles(i).turn(turn);
          particles(i).move(move);
          if particles(i).insideMap == 0 %check if the particles are in the map
              particles(i).randomPose(0);
          end
    end
    
    
end

end