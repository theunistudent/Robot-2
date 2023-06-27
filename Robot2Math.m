% need to figure out how to move accross for the other 10 carracters
%refference one spot.

clc
clear

% Author: Raghav Hariharan
% For MTRN4230 2023

%% ----- EXAMPLE 7: Drawing Letters -----
% Example of drawing a letter/digit from the rvc toolbox hershey library
clear all;

startup_rvc; % Startup the rvc toolbox


load hershey; % Load in the hershey fonts


input = ['22','-','2']
soltution = str2num(input)

%creates empty second argument
secondArgument = [];

% Splitting up into sepperate components 
operation = false;
for i = 1:length(input)
    if operation == true
    secondArgument = input(i:length(input));
    break
    end
    if input(i) == '+' || input(i) == '-' || input(i) == '*'
        operation = true;
        operator = input(i);
    end
    if operation == false
        firstArgument(i) = input(i);
    end 
end
    

firstArgument
secondArgument
operator

%sudo code
%load everything in as one long thing
%when outputing use a counter to now what you are up to
%use transforms to move it around.

allCharecters = [input, firstArgument, secondArgument, operator, num2str(soltution)];




%define wrighting hight
h = 30;

%part 2 variables
deltaTheta = 0;
deltaX = 0;
deltaY = 0;

% gets rotaions and transpose for part 1
T1 = SE2((-350), (-588.53), (-90), 'deg');
T1 = SE2((-deltaY), (deltaX), (deltaTheta), 'deg')*T1;

traj_storage = cell(1:length(allCharecters));

scale = 0.04; % Select the scale of the digit. 1 = 100%, 0.1 = 10% scale

for i = 1:length(allCharecters) 
    character = hershey{allCharecters(i)}; % Select the letter that you want to draw (Letter or number works)
    path = [scale*character(1).stroke; zeros(1,numcols(character.stroke))]; % create the path 
    
    
    % Where ever there is an nan it indicates that we need to lift up.
    k = find(isnan(path(1,:)));
    
    % At these positions add in a z hight
    path(:,k) = path(:,k-1); path(3,k) = 0.25*scale; % Determine the hight of the lift up motions. 0.2 * scale is the height. 0.2 is in m
    
    traj = [path'*1000]; % convert to the mm units so that we can use the rtde toolbox
    
    traj_storage{i} = traj;

end

% Generate a plot of what we are expecting
%scatter3(traj(:,1), traj(:,2), traj(:,3));
%plot3(traj(:,1), traj(:,2), traj(:,3));



%% NOW USE THE RTDE TOOLBOX TO EXECUTE THIS PATH!


% % TCP Host and Port settings
host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.230.128'; % THIS IP ADDRESS MUST BE USED FOR THE VMWARE
 %host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
% 

% Calling the constructor of rtde to setup tcp connction
rtde = rtde(host,port);

% Setting home
home = [-588.53, -133.30, 371.91, 2.2214, -2.2214, 0.00];

%trying to apply translation and rotation
xy = [home(1:2)';1];
inverse = inv(T1.T);
P1 = inverse*xy;


poses = rtde.movej(home);

pathStorage = {};

    path = [];


% setting move parameters
v = 0.5;
a = 1.2;
blend = 0.005;

% counters
    k = 0;
    l = 0;
    w = 0;
    center = 0;

% for loop for cell
for j = 1:length(allCharecters)
    traj = traj_storage{j};

    % create offset to space letters

        %change Y offset
    if j == length(input)+1
        k = k + 1;
        w = 0;
    elseif j == length(input)+length(firstArgument)+1
        k = k + 1;
        w = 0;
    elseif j == length(input)+length(firstArgument)+length(secondArgument)+length(operator)+1
        k = k + 1;
        w = 0;
    end


    
    % invcrement l for the X offset
    if j < length(input)+1
        l = l + 1;
        if j == length(firstArgument)+1
            center = l;
        end
        
    end
    % x offset for first argument
    if j > length(input) && j <= length(input)+length(firstArgument)
        w = w + 1;
        l = center - length(firstArgument)+w;
        
    end
    %for second argument and operator
    if j > length(input)+length(firstArgument) && j <= length(input)+length(firstArgument)+length(secondArgument)+length(operator)
        w = w + 1;
        l = center - length(secondArgument) + w;
        
        
    end
    
    if j > length(input)+length(firstArgument)+length(secondArgument)+length(operator)
        center
        w = w + 1
        l = center - length(num2str(soltution))+w
    end
   



% center
% l
% w
% k


    offsetX = 40*(l-1);
    

    offsetY = 40*(k-1);



    %moving up between letters
    point = [-700+offsetX, -133.30-offsetY, 60, 2.2214, -2.2214, 0.00,a,v,0,blend];
    xy = [point(1:2),1]';
    P1 = inverse*xy;
    point = [P1(1:2)', point(3:10)];
    path = cat(1,path,point);

    % Populate the path array
    for i = 1:size(traj,1)
        %disp(i);
        %disp(traj(i,1:3) + [-588.53, -133.30 100]);
        


        point = [[(traj(i,1:3) + [-700+offsetX, -133.30-offsetY h]),(home(4:6))],a,v,0,blend];
        xy = [point(1:2),1]';
        P1 = inverse*xy;
        point = [P1(1:2)', point(3:10)];
        if isempty(path)
            path = point;
        else
            path = cat(1,path,point);
        end

    end


end
    
%rotating the letters
% theta = 90;
% R = [cosd(theta) -sind(theta) ; sind(theta) cosd(theta)];
% rotpoint = R*path(1:2,:);
% path(1:2,:) = rotpoint;

%excute movment
%path(:,1:3) = transpose(rotz(pi/2)*path(:,1:3)');

poses = rtde.movej(path);

rtde.movej(home);


hold on
rtde.drawPath(poses);




rtde.close;