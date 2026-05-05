% WRITE YOUR NAME: Anwitha Sanivarapu

%% Load in Crash Data
CrashData = readtable('VWID4_Data.csv');

%% Solving the model at 35 mph example and comparison figure produced:
% Refer to this initial code for help with unit conversions/interpreting
% ode23 function call
global xmax max_acc
xmax = 0;
max_acc = 0;
x0 = 0;
v0 = 35;%mph
v0 = v0/3600*5280*12*2.54/100; %m/s
[t,y] = ode23(@linearInelastic_spring,[0 0.200], [x0;v0], odeset('RelTol',1e-8));
dp = y(:,1)*100/2.54; %in
vel = y(:,2)*3600/5280/12/2.54*100; %mph

%% Plot a comparison
figure()
hold on
plot(t, vel,...
    'LineWidth', 2, ...
    'DisplayName', 'Model Velocity (mph)')
plot(CrashData.Time, CrashData.Velocity_mks*3600/5280/12/2.54*100,...
    'LineWidth', 2, ...
    'DisplayName', 'Actual Velocity (mph)')
plot(t, dp,...
    'LineWidth', 2, ...
    'DisplayName', 'Model Displacement (in)')
plot(CrashData.Time, CrashData.Disp_mks*100/2.54,...
    'LineWidth', 2, ...
    'DisplayName', 'Actual Displacement (in)')
xlim([0,0.2])
ylim([-10,40])
xlabel('Time (s)', 'fontsize', 12)
ylabel('Speed (mph), Displacement (in)', 'fontsize', 12)
legend('show', 'Location', 'northeast')
hold off

%% Analysis of Unbelted Occ at 25, 30, 35 mph (YOUR PART)
figure
speed = [25, 30, 35]; %mph
% Initializing vectors containing answers for Questions 3 and 4
% (Values must be assigned to these within following for loop)
peak_disp = zeros(3,1);
peak_acc = zeros(3,1);
reb_vel = zeros(3,1);
steer_wheel_strike = zeros(3,1);

% Start of for loop iterating through car speed cases (25, 30, and 35 mph)
for i = 1:length(speed)
    % Reseting global variables to 0 before next speed calculation:
    xmax = 0;
    max_acc = 0;
    
    % Specify initial conditions for current iteration of ODE calculation:
    x0 = 0; % initial displacement (m)
    v0 = speed(i); % initial velocity (mph)
    
    % Next line converts velocity from mph to m/s
    v0 = v0/3600*5280*12*2.54/100; % initial velocity (m/s)
    
    % Use 'ode23' to call your COMPLETE 'linearInelastic_spring' function and 
    % solve the current set of ODEs.
    init_cond = [x0;v0];
    % Use a tspan of [0 0.200].
    tspan = [0 0.200];
    % Add the condition 'odeset('RelTol',1e-8)' AFTER your tspan and initial conditions.
    error = odeset('RelTol',1e-8);
    % This condition sets an acceptable error for 'ode23'.
    [t,y] = ode23(@linearInelastic_spring,tspan, init_cond, error);
    
    % Calculate the OCCUPANT displacement in INCHES from the outputs of ode23:
    dp = y(:,1)*100/2.54; %in
    
    % Calculate the OCCUPANT velocity in MPH from the outputs of ode23:
    vel = y(:,2)*3600/5280/12/2.54*100; %mph
    
    % Assign each of the following to the appropriate vectors:
    % Peak Displacement in INCHES
    % Peak Acceleration in G's (1 G = 9.81 m/s^2)
    % Rebound Velocity in MPH (HINT: rebound is after crash and negative)
    peak_disp(i) = max(dp); %in
    peak_acc(i) = abs(max_acc / 9.81); %G
    reb_vel(i) = abs(min(vel)); %mph
    
    % Calculate and assign the speed at which the occupant strikes the 
    % steering wheel for each case.
    % (HINT: consider using the MATLAB 'interp1' function to perform interpolation.
    value = 15;
    loading_phase = y(:,2) >= 0;
    dp_loading = dp(loading_phase);
    vel_loading = vel(loading_phase);
    steer_wheel_strike(i) = interp1(dp_loading, vel_loading, value);
    
    hold on
    plot(dp, vel,...
        'LineWidth', 2, ...
        'DisplayName', sprintf('%0.0f mph', speed(i))) 
    xlim([0,25])
    ylim([0, 50])
    xlabel('Occupant Displacement w.r.t Vehicle (in)', 'fontsize', 12)
    ylabel('Occupant Velocity w.r.t. Vehicle (mph)', 'fontsize', 12)
    legend('show', 'Location', 'southeast')
    hold off

end

%% Function modeling car as inelastic spring (Covered in Class - YOU MUST FILL IN)
function xprime = linearInelastic_spring(t,x)
% Input Variables: 
% t         :       time vector
% x         :       2x1 vector containing displacement and velocity
%                   x = [displacement;
%                        velocity]
% Outpu Variables:
% xprime    :       2x1 time derivative vector of 'x' input
%                   xprime = [velocity;
%                             acceleration]
    global xmax max_acc
    mass = 2305;
    kload = 993462;
    kunload = 18021775;
    % Initialize force as zero, but calculate actual value in latter lines depending
    % on conditions.
   force = 0;

    % Loading phase: moving forward, compressing
    if x(1) > 0 && x(2) > 0
        force = kload * x(1);
    end

    % Unloading phase: moving backward, rebounding
    if x(1) > 0 && x(2) < 0
        force = kunload * (x(1) - xmax);
    end

    % Separation phase: no contact
    if force < 0 
        force = 0;
    end

    % Track max displacement
    if xmax < x(1)
        xmax = x(1);
    end

    % Compute acceleration
    a = force / mass;

    % Track max acceleration
    if abs(a) > abs(max_acc)
        max_acc = a;
    end

    % Return derivatives as column vector
    xprime = [x(2); a];
end