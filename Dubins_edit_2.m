function path = Dubins_edit_2()

init_place = [2.8*rand-1.4, 2.8*rand-1.4, 2*pi*rand-pi];
last_place = [2.8*rand-1.4, 1.5 , pi/2];

%this line for the parking phase of the vehicle.
last_place = [last_place(1), last_place(2)-0.25, last_place(3)];

radius = 1.5;

%a_time = tic;
path = dubins_curve(init_place, last_place, radius, 0.1);

while ((min(path(:,1))<-1.4 || min(path(:,2))<-1.4 || max(path(:,1))>1.4 || max(path(:,2))>1.4) == 1)
    path = dubins_curve(init_place, last_place, radius, 0.1);
    radius = radius - 0.1;
   
    if(radius < 0.25)        
        break
    end  
end

% b_time = toc(a_time);
% x1 = path(:,1);
% y1 = path(:,2);
% thetas = path(:,3);
% leng = zeros(length(x1)-1,1);
% 
% for ind = 1:length(x1)-1   
%     leng(ind) = sqrt((x1(ind)-x1(ind+1)).^2+(y1(ind)-y1(ind+1)).^2);  
% end
% 
% % leng(:,1) = 0.1; %fixed vector length
% u1 = leng .* cos(thetas(1:length(thetas)-1)); 
% v1 = leng .* sin(thetas(1:length(thetas)-1));
% figure;
% quiver(x1(1:length(x1)-1),y1(1:length(y1)-1),u1,v1,'LineWidth',2);
% title(['calculation time = ',num2str(b_time),'s']);
% xlim([-1.5,1.5]);
% ylim([-1.5,1.5]);
% grid minor

    

%dubins_curve(init_place, last_place, radius, 0.1);

% Input: 
%   p1/p2: Initial and ending 2-D pose
%          In row vectors, e.g. [x, y, theta]
%   r: turning radius of the curve
% Output: 
%   param: a struct that includes 4 field:
%     p_init: Initial pose, equals to input p1
%     type: One of the 6 types of the dubins curve
%     r: Turning radius, same as input r, also the scaling factor for 
%        the dubins paramaters
%     seg_param: angle or normalized length, in row vector [pr1, pr2, pr3]
% Reference:
% 	https://github.com/AndrewWalker/Dubins-Curves#shkel01
%   Shkel, A. M. and Lumelsky, V. (2001). "Classification of the Dubins
%                 set". Robotics and Autonomous Systems 34 (2001) 179¡V202
function path = dubins_curve(p1, p2, r, stepsize, quiet)
    
    %%%%%%%%%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % there are 6 types of dubin's curve, only one will have minimum cost
    % LSL = 1;
	% LSR = 2;
	% RSL = 3;
	% RSR = 4;
	% RLR = 5;
    % LRL = 6;
    
    % The three segment types a path can be made up of
    % L_SEG = 1;
    % S_SEG = 2;
    % R_SEG = 3;

    % The segment types for each of the Path types
    %{
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
                L_SEG, S_SEG, R_SEG ;...
                R_SEG, S_SEG, L_SEG ;...
                R_SEG, S_SEG, R_SEG ;...
                R_SEG, L_SEG, R_SEG ;...
                L_SEG, R_SEG, L_SEG ]; 
    %}
            
    % the return parameter from dubins_core
    % param.p_init = p1;              % the initial configuration
    % param.seg_param = [0, 0, 0];    % the lengths of the three segments
    % param.r = r;                    % model forward velocity / model angular velocity turning radius
    % param.type = -1;                % path type. one of LSL, LSR, ... 
    %%%%%%%%%%%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Handle inputs.
    if nargin < 3
        error('Function requires at least two inputs.');
    elseif nargin < 4
        stepsize = 0;
    elseif nargin < 5 
        quiet = 0;  %Default/undefined is not quiet
    end
    
    if ~quiet
        close(findobj('type','figure','name','Dubins curve'));
        tic;
    end
    
    % Check if the destination lies on the circle that can be reach
    % directly from the starting point
    if(p1(3)~=p2(3))
        T = [cos(p1(3)), sin(p1(3));...
             cos(p2(3)), sin(p2(3)) ];
        Y = [p1(1)*cos(p1(3)) + p1(2)*sin(p1(3)); ...
             p2(1)*cos(p2(3)) + p2(2)*sin(p2(3)) ];
        X = T \ Y;
        if( norm(X-reshape(p1(1:2), [2,1]),2) == r ) && ( norm(X-reshape(p2(1:2),[2,1]),2) == r )
            %warning('p2 lies on the turning circle from the p2, dubins curve may be suboptimal');
        end
    end
        
    
    % main function
    param = dubins_core(p1, p2, r);
    if stepsize <= 0
        stepsize = dubins_length(param)/1000;
    end
    path = dubins_path_sample_many(param, stepsize);
     
    % plot if not quiet
    if ~quiet
        disp('dubins calculation time'); toc;
        % plotting
        tic;    % most of the time is spent on plotting
%         figure('name','Dubins curve');
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
%         text(p1(1), p1(2),'start','HorizontalAlignment','center');
%         text(p2(1), p2(2),'end','VerticalAlignment','top');
        disp('plot drawing time'); toc;
    end
end

function path = dubins_path_sample_many( param, stepsize)
    if param.flag < 0
        path = 0;
        return
    end
    length = dubins_length(param);
    path = -1 * ones(floor(length/stepsize), 3);
    x = 0;
    i = 1;
    while x <= length
        path(i, :) = dubins_path_sample( param, x );
        x = x + stepsize;
        i = i + 1;
    end
    return
end

function length = dubins_length(param)
    length = param.seg_param(1);
    length = length + param.seg_param(2);
    length = length + param.seg_param(3);
    length = length * param.r;
end


%{
 * Calculate the configuration along the path, using the parameter t
 *
 * @param path - an initialised path
 * @param t    - a length measure, where 0 <= t < dubins_path_length(path)
 * @param q    - the configuration result
 * @returns    - -1 if 't' is not in the correct range
%}
function end_pt = dubins_path_sample(param, t)
    if( t < 0 || t >= dubins_length(param) || param.flag < 0)
        end_pt = -1;
        return;
    end

    % tprime is the normalised variant of the parameter t
    tprime = t / param.r;

    % In order to take rho != 1 into account this function needs to be more complex
    % than it would be otherwise. The transformation is done in five stages.
    %
    % 1. translate the components of the initial configuration to the origin
    % 2. generate the target configuration
    % 3. transform the target configuration
    %      scale the target configuration
    %      translate the target configration back to the original starting point
    %      normalise the target configurations angular component

    % The translated initial configuration
    p_init = [0, 0, param.p_init(3) ];
    
    %%%%%%%%%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % The three segment types a path can be made up of
    L_SEG = 1;
    S_SEG = 2;
    R_SEG = 3;

    % The segment types for each of the Path types
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
                L_SEG, S_SEG, R_SEG ;...
                R_SEG, S_SEG, L_SEG ;...
                R_SEG, S_SEG, R_SEG ;...
                R_SEG, L_SEG, R_SEG ;...
                L_SEG, R_SEG, L_SEG ]; 
    %%%%%%%%%%%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Generate the target configuration
    types = DIRDATA(param.type, :);
    param1 = param.seg_param(1);
    param2 = param.seg_param(2);
    mid_pt1 = dubins_segment( param1, p_init, types(1) );
    mid_pt2 = dubins_segment( param2, mid_pt1,  types(2) );
    
    % Actual calculation of the position of tprime within the curve
    if( tprime < param1 ) 
        end_pt = dubins_segment( tprime, p_init,  types(1) );
    elseif( tprime < (param1+param2) ) 
        end_pt = dubins_segment( tprime-param1, mid_pt1,  types(2) );
    else 
        end_pt = dubins_segment( tprime-param1-param2, mid_pt2,  types(3) );
    end

    % scale the target configuration, translate back to the original starting point
    end_pt(1) = end_pt(1) * param.r + param.p_init(1);
    end_pt(2) = end_pt(2) * param.r + param.p_init(2);
    end_pt(3) = mod(end_pt(3), 2*pi);
    return;
end

%{
 returns the parameter of certain location according to an inititalpoint,
 segment type, and its corresponding parameter
%}
function seg_end = dubins_segment(seg_param, seg_init, seg_type)
    global seg_end;
    L_SEG = 1;
    S_SEG = 2;
    R_SEG = 3;
    if( seg_type == L_SEG ) 
        seg_end(1) = seg_init(1) + sin(seg_init(3)+seg_param) - sin(seg_init(3));
        seg_end(2) = seg_init(2) - cos(seg_init(3)+seg_param) + cos(seg_init(3));
        seg_end(3) = seg_init(3) + seg_param;
    elseif( seg_type == R_SEG )
        seg_end(1) = seg_init(1) - sin(seg_init(3)-seg_param) + sin(seg_init(3));
        seg_end(2) = seg_init(2) + cos(seg_init(3)-seg_param) - cos(seg_init(3));
        seg_end(3) = seg_init(3) - seg_param;
    elseif( seg_type == S_SEG ) 
        seg_end(1) = seg_init(1) + cos(seg_init(3)) * seg_param;
        seg_end(2) = seg_init(2) + sin(seg_init(3)) * seg_param;
        seg_end(3) = seg_init(3);
    end
end

function param = dubins_core(p1, p2, r)
    %%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%
    % Here are some usefuldefine headers for better implementation
    % there are 6 types of dubin's curve, only one will have minimum cost
    % LSL = 1;
	% LSR = 2;
	% RSL = 3;
	% RSR = 4;
	% RLR = 5;
    % LRL = 6;
    
    % The three segment types a path can be made up of
    % L_SEG = 1;
    % S_SEG = 2;
    % R_SEG = 3;

    % The segment types for each of the Path types
    %{
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
                L_SEG, S_SEG, R_SEG ;...
                R_SEG, S_SEG, L_SEG ;...
                R_SEG, S_SEG, R_SEG ;...
                R_SEG, L_SEG, R_SEG ;...
                L_SEG, R_SEG, L_SEG ]; 
    %}
    %%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%
            
    % the return parameter
    param.p_init = p1;              % the initial configuration
    param.seg_param = [0, 0, 0];    % the lengths of the three segments
    param.r = r;                    % model forward velocity / model angular velocity turning radius
    param.type = -1;                % path type. one of LSL, LSR, ... 
    param.flag = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%% START %%%%%%%%%%%%%%%%%%%%%%%%%
    % First, basic properties and normalization of the problem
    dx = p2(1) - p1(1);
    dy = p2(2) - p1(2);
    D = sqrt( dx^2 + dy^2 );
    d = D / r;                  % distance is shrunk by r, this make lengh calculation very easy
    if( r <= 0 )
        param.flag = -1;
        return;
    end
    theta = mod(atan2( dy, dx ), 2*pi);
    alpha = mod((p1(3) - theta), 2*pi);
    beta  = mod((p2(3) - theta), 2*pi);

    % Second, we find all possible curves
    global test_param;
    best_word = -1;
    best_cost = -1;
    test_param(1,:) = dubins_LSL(alpha, beta, d);
    test_param(2,:) = dubins_LSR(alpha, beta, d);
    test_param(3,:) = dubins_RSL(alpha, beta, d);
    test_param(4,:) = dubins_RSR(alpha, beta, d);
    test_param(5,:) = dubins_RLR(alpha, beta, d);
    test_param(6,:) = dubins_LRL(alpha, beta, d);
    
    for i = 1:1:6
        if(test_param(i,1) ~= -1) 
            cost = sum(test_param(i,:));
            if(cost < best_cost) || (best_cost == -1)
                best_word = i;
                best_cost = cost;
                param.seg_param = test_param(i,:);
                param.type = i;
            end
        end
    end

    if(best_word == -1) 
        param.flag = -2;             % NO PATH
        return;
    else
        return;
    end
end

function param1 = dubins_LSL(alpha, beta, d)
    
    tmp0 = d + sin(alpha) - sin(beta);
    p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(alpha) - sin(beta)));
    if( p_squared < 0 )
        param1 = [-1, -1, -1];
        return;
    else
        tmp1 = atan2( (cos(beta)-cos(alpha)), tmp0 );
        t = mod((-alpha + tmp1 ), 2*pi);
        p = sqrt( p_squared );
        q = mod((beta - tmp1 ), 2*pi);
        param1(1) = t; 
        param1(2) = p; 
        param1(3) = q;
        return ;
    end
end
function param2 = dubins_LSR(alpha, beta, d)
    
    p_squared = -2 + (d*d) + (2*cos(alpha - beta)) + (2*d*(sin(alpha)+sin(beta)));
    if( p_squared < 0 )
        param2 = [-1, -1, -1];
        return;
    else
        p    = sqrt( p_squared );
        tmp2 = atan2( (-cos(alpha)-cos(beta)), (d+sin(alpha)+sin(beta)) ) - atan2(-2.0, p);
        t    = mod((-alpha + tmp2), 2*pi);
        q    = mod(( -mod((beta), 2*pi) + tmp2 ), 2*pi);
        param2(1) = t; 
        param2(2) = p; 
        param2(3) = q;
        return ;
    end
end
function param3 = dubins_RSL(alpha, beta, d)
    
    p_squared = (d*d) -2 + (2*cos(alpha - beta)) - (2*d*(sin(alpha)+sin(beta)));
    if( p_squared< 0 ) 
        param3 = [-1, -1, -1];
        return;
    else
        p    = sqrt( p_squared );
        tmp2 = atan2( (cos(alpha)+cos(beta)), (d-sin(alpha)-sin(beta)) ) - atan2(2.0, p);
        t    = mod((alpha - tmp2), 2*pi);
        q    = mod((beta - tmp2), 2*pi);
        param3(1) = t;
        param3(2) = p; 
        param3(3) = q;
        return ;
    end
end
function param4 = dubins_RSR(alpha, beta, d)
    
    tmp0 = d-sin(alpha)+sin(beta);
    p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(beta)-sin(alpha)));
    if( p_squared < 0 )
        param4 = [-1, -1, -1];
        return;
    else
        tmp1 = atan2( (cos(alpha)-cos(beta)), tmp0 );
        t = mod(( alpha - tmp1 ), 2*pi);
        p = sqrt( p_squared );
        q = mod(( -beta + tmp1 ), 2*pi);
        param4(1) = t; 
        param4(2) = p; 
        param4(3) = q;
        return;
    end
end
function param5 = dubins_RLR(alpha, beta, d)
    
    tmp_rlr = (6. - d*d + 2*cos(alpha - beta) + 2*d*(sin(alpha)-sin(beta))) / 8.;
    if( abs(tmp_rlr) > 1)
        param5 = [-1, -1, -1];
        return;
    else
        p = mod(( 2*pi - acos( tmp_rlr ) ), 2*pi);
        t = mod((alpha - atan2( cos(alpha)-cos(beta), d-sin(alpha)+sin(beta) ) + mod(p/2, 2*pi)), 2*pi);
        q = mod((alpha - beta - t + mod(p, 2*pi)), 2*pi);
        param5(1) = t;
        param5(2) = p;
        param5(3) = q;
        
        return;
    end
end
function param6 = dubins_LRL(alpha, beta, d)
    
    tmp_lrl = (6. - d*d + 2*cos(alpha - beta) + 2*d*(- sin(alpha) + sin(beta))) / 8.;
    if( abs(tmp_lrl) > 1)
        param6 = [-1, -1, -1]; return;
    else
        p = mod(( 2*pi - acos( tmp_lrl ) ), 2*pi);
        t = mod((-alpha - atan2( cos(alpha)-cos(beta), d+sin(alpha)-sin(beta) ) + p/2), 2*pi);
        q = mod((mod(beta, 2*pi) - alpha -t + mod(p, 2*pi)), 2*pi);
        param6(1) = t;
        param6(2) = p;
        param6(3) = q;
        return;
    end
end

end