clear all
clc

failed_path = 0;
succesfull_path = 0;

% numofCore=5;
% poolobj = gcp('nocreate');
% if(isempty(poolobj))
%    parpool(numofCore)
% else
%     delete(poolobj);
%    parpool(numofCore)
% end

for ind=1:100
    
   init_place = [2.5*rand-1.25, 2.5*rand-1.25, 2*pi*rand-pi];
   last_place = [2.8*rand-1.4, 1.25 , pi/2];
   path = Dubins_edit_2(init_place,last_place);
    
   
   if(max(abs(path(:,1)))>1.45 || max(abs(path(:,2))) > 1.45)
       
       figure;
       
        p1 = [path(1,1),path(1,2),path(1,3)];
        p2 = [path(length(path),1),path(length(path),2),path(length(path),3)];
        
        subplot(1,3,1);
        
        plot(path(:,1), path(:,2),'LineWidth',2); axis equal; hold on
        xlim([-1.55,1.55])
        ylim([-1.55,1.55])
        scatter(p1(1), p1(2), 45, '*','r','LineWidth',1); hold on;
        grid minor

        theta = p1(3);
        len = 0.5; % magnitude (length) of arrow to plot
        x = p1(1); y = p1(2);
        u = len * cos(theta); % convert polar (theta,r) to cartesian
        v = len * sin(theta);
        quiver(x,y,u,v, "LineWidth", 2);

        scatter(p2(1), p2(2), 45, 'square','b','LineWidth',1); hold on;
        %text(p1(1), p1(2),'start','HorizontalAlignment','center');
        %text(p2(1), p2(2),'end','VerticalAlignment','top');
        
        
       
       last_place2 = [0,0,0];
       path1 = Dubins_edit_2(init_place,last_place2);
       
        p1 = [path1(1,1),path1(1,2),path1(1,3)];
        p2 = [path1(length(path1),1),path1(length(path1),2),path1(length(path1),3)];
        
       
        subplot(1,3,2); 
        plot(path1(:,1), path1(:,2),'LineWidth',2); axis equal; hold on
        xlim([-1.55,1.55])
        ylim([-1.55,1.55])
        scatter(p1(1), p1(2), 45, '*','r','LineWidth',1); hold on;
        grid minor

        theta = p1(3);
        len = 0.5; % magnitude (length) of arrow to plot
        x = p1(1); y = p1(2);
        u = len * cos(theta); % convert polar (theta,r) to cartesian
        v = len * sin(theta);
        quiver(x,y,u,v, "LineWidth", 2);

        scatter(p2(1), p2(2), 45, 'square','b','LineWidth',1); hold on;
        %text(p1(1), p1(2),'start','HorizontalAlignment','center');
        %text(p2(1), p2(2),'end','VerticalAlignment','top');
        
        path2 = Dubins_edit_2(last_place2,last_place);
        
        path = cat(1,path1,path2);
        
        
        p1 = [path(1,1),path(1,2),path(1,3)];
        p2 = [path(length(path),1),path(length(path),2),path(length(path),3)];
        
        subplot(1,3,3);
        
        plot(path(:,1), path(:,2),'LineWidth',2); axis equal; hold on
        xlim([-1.55,1.55])
        ylim([-1.55,1.55])
        scatter(p1(1), p1(2), 45, '*','r','LineWidth',1); hold on;
        grid minor

        theta = p1(3);
        len = 0.5; % magnitude (length) of arrow to plot
        x = p1(1); y = p1(2);
        u = len * cos(theta); % convert polar (theta,r) to cartesian
        v = len * sin(theta);
        quiver(x,y,u,v, "LineWidth", 2);

        scatter(p2(1), p2(2), 45, 'square','b','LineWidth',1); hold on;
        %text(p1(1), p1(2),'start','HorizontalAlignment','center');
        %text(p2(1), p2(2),'end','VerticalAlignment','top');
          
           if(max(abs(path(:,1)))>1.45 || max(abs(path(:,2))) > 1.45)
                failed_path = failed_path +1;
           else
               succesfull_path = succesfull_path +1;
           end
     
      
       
   else
       succesfull_path = succesfull_path +1;
   end
   
   
%% plotting code
%    p1 = [path(1,1),path(1,2),path(1,3)];
%         p2 = [path(length(path),1),path(length(path),2),path(length(path),3)];
%         
%         %subplot(10,10,ind);
%         figure;
%         plot(path(:,1), path(:,2),'LineWidth',2); axis equal; hold on
%         xlim([-1.55,1.55])
%         ylim([-1.55,1.55])
%         scatter(p1(1), p1(2), 45, '*','r','LineWidth',1); hold on;
%         grid minor
% 
%         theta = p1(3);
%         len = 0.5; % magnitude (length) of arrow to plot
%         x = p1(1); y = p1(2);
%         u = len * cos(theta); % convert polar (theta,r) to cartesian
%         v = len * sin(theta);
%         quiver(x,y,u,v, "LineWidth", 2);
% 
%         scatter(p2(1), p2(2), 45, 'square','b','LineWidth',1); hold on;
%         %text(p1(1), p1(2),'start','HorizontalAlignment','center');
%         %text(p2(1), p2(2),'end','VerticalAlignment','top');
%         
%     
    
end

%delete(gcp);

