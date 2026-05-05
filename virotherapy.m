% Code for Script to Call your Local Function
global alpha delta mu theta_0 beta gamma b
% Input starting and final time to define the timespan

% Input given parameters
alpha = 2*10^-1; % proliferation rate of cancer cells, 1/h
delta = 1/18; % death rate of infected cancer cells, 1/h
mu = 1/48; % removal rate of debris of dead cells, 1/h
theta_0 = 10^6; % total cell capacity, cells/mm^3
beta = 7*10^-8; % rate of infection of cancer cells by viruses, mm^3/h/virus
gamma = 2.5 *10^-2; % clearance rate of virus particles, 1/h
b = 500; % replication number of a virus at the time of death of the infected cancer cell

% Input initial conditions and sort them into a single vector 'w_ini'
x0 =  8*10^5; % cells/mm^3
y0 = 10^5; % cells/mm^3
n0 = theta_0 - x0 - y0; % remaining healthy cells
v0 = 10^6; % virus/mm^3
R0 = 2; % mm

w_ini = [x0; y0; n0; v0; R0];

% Define timespan
tspan = [0 20];

% Implement ode45
[t, w] = ode45(@viro_model, tspan, w_ini);

% Plot solutions
figure;
hold on;
plot(t, w(:,1), 'r'); % Cancer cells
plot(t, w(:,2), 'b'); % Infected cancer cells
plot(t, w(:,3), 'g'); % Debris
plot(t, w(:,4), 'r--'); % Virus particles
plot(t, w(:,5), 'b--'); % Tumor radius
xlabel('Time (hours)');
ylabel('Tumor Size');
legend('Cancer Cells', 'Infected Cancer Cells', 'Debris', 'Virus', 'Tumor Radius');
title('Virotherapy Model When b is 500');
hold off;

% Code for Local Function
function dy = viro_model(~, w)
    global alpha delta mu theta_0 beta gamma b

    % Extract variables
    x = w(1);   
    y = w(2);  
    n = w(3);  
    v = w(4);  
    R = w(5);

    % Compile differential equations
    dy = zeros(5,1);
    dy(1) = alpha*x - beta*x*v; % dx/dt
    dy(2) = beta*x*v - delta*y; % dy/dt
    dy(3) = delta*y - mu*n; % dn/dt
    dy(4) = b*delta*y - gamma*v; % dv/dt
    dy(5) = (R / (3 * theta_0)) * (alpha*x - mu*(theta_0 - x - y)); % dR/dt
end
