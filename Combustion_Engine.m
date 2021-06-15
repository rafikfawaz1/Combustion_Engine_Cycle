%% Workspace clear
clear all;
clc;
close all;
%% Parameters 
warning('off','all')
steps        = 1000;                  % Calculation-steps per engine revolution       [-]
engine_speed = 3000;                  % Rotational speed per minute                   [rpm]
stepsize  = (60/engine_speed)/steps;  % Simulink calculation steps                    [s]
sim_time  = stepsize*(2*steps-1);     % Two engine revolutions, one cycle             [s]
R           = 287;                    % Specific Universal gas constant               [J/kg.K]
Rho         = 1.27;                   % Density of air                                [kg/m^3]
gma         = 1.35;                   % Specific heat ratio                           [-]    
MAP         = 80000;                  % Manifold Absolute Pressure                    [Pa]
P_ambient   = 100000;                 % Ambient pressure                              [Pa]
P_exhaust   = 120000;                 % Pressure in cylindre during exhaust stroke    [Pa]
CR          = 10;                     % Compression ratio                             [-]
Bore        = 82.7*10^-3;             % Piston bore                                   [m]
Stroke      = 93*10^-3;               % Piston stroke                                 [m]
V_cyl       = (Bore/2)^2*pi*Stroke;   % cylindre volume                               [m^3]
V_comp      = V_cyl/(CR-1);           % Compression ratio                             [-]
CrankRadius = Stroke/2;               % Crank radius                                  [m]
ConRod      = 140*10^-3;              % Length of conrod                              [m]
Valve_lift  = 9*10^-3;                % Valve lift                                    [m]
Nr_of_valves= 2;                      % Number of valves                              [-]
Valve_dia   = 30*10^-3;               % Valve diameter                                [m]
Cd          = 0.35;                   % Flow discharge coefficient over intake valve  [-]
P_0         = 100000;                 % Atmosferic presure at start of intake stroke  [Pa]
T_air       = 323;                    % Air temperature at start of intake stroke     [Celcius]
m_0         =P_0*(V_comp)/(R*T_air);  % Mass of air inside cylinder at intake start   [moles]
V_sonic     = 20.0457*sqrt(T_air);    % Limited air flow of intake "sonic speed"      [m/s]
V1          = V_comp+V_cyl;           % Cylindre volume at BDC of piston              [m^3]
V2          = V_comp;                 % Cylindre volume at TDC of piston              [m^3]
V3          = V2;                     % Equal volume for compression and combustion   [m^3]
V4          = V1;                     % Equal volume for intake and exhaust           [m^3]
P_start     = m_0*R*T_air/V2;         % Start pressure inside cylindre                [Pa]
LHV         =44400*10^3;              % Energy content                                [J/kg]
AFR         =14.7;                    % actual Air Fuel ratio                         [kg/kg]
AFR_stoich  =14.7;                    % Stoichiometric air fuel ratio                 [kg/kg]
theta0_to_180=linspace(0 , 180 ,500);
%% Simulate model
sim('Intake')                         % Simulation of model to get initial pressure values  
P_1    =    max(P_Intake);            % Pressur at the start of Compression
P_2    =    max(P_end_compression);   % pressure end of compression
P_3    =    max(P_end_combustion);    % Pressur at the end of Combustion
C2     =    P_1*max(V_Intake).^gma ;  % Isometric compression
C3     =    P_3*V3.^gma;              % Isometric expansion 
%% Intakestroke      (pressure constant)
P_Start     =   P_Intake(1);          % Startpoint of intakestroke                                       
Mass_added = max(Mass_added_simulink);% Addes mass at end of intakestroke
T1     =    T_air;                    % Temperature at end of intakestroke
%% Compressiostroke  (pressure Isentropic)                        
T2 = nthroot(((T_air^(gma/(gma-1)))*(V1/V2)^gma),(gma/(gma-1))); % End of compressionstroke
m_end_comp= (P_2*V2)/(R*T2);           % Mass Trapped at end of compression 
%% Cumbustionstroke  (pressure Isochore) 
T3      =   T2*(P_3/P_2);             % Temperatuers at the end of Combustion, strating of expansion
%% work              (pressure Isentropic)                   
P_4     =   P_3*(V3/V4)^gma;          % Pressure at the end of expansion, beginning of exhauststroke
T4=nthroot(((T3^(gma/(gma-1)))*(V2/V1)^gma),(gma/(gma-1)));     % Temperatures at the end of exhauststroke
m_end_exp= (P_4*V4)/(R*T4);            % Mass trapped at end of Expansion
%% Exhaust           (presure constant)
V5      =   V_comp;                   % Volume at end of exhauststroke
P_5     =   P_4;                      % Exhaust is at constant pressure
%% Dynamic Compression 
V_compression = piston_kinematics(Bore,Stroke,ConRod,CR,180,360);% Volume compression related to crankangle
P_compression = C2./V_compression.^gma;                          % Pressure calculated with isentropic calculation
%% Dynamic Expansion (work)
V_expansion = piston_kinematics(Bore,Stroke,ConRod,CR,360,540);  % Volume compression related to crankangle
P_expansion = C3./V_expansion.^gma;                              % Pressure calculated with isentropic calculation
%% p-V diagram Plot
figure
sgtitle('p-V diagram');
hold on; 
plot(V_Intake,P_Intake,'linewidth',2,'color', 'b');
plot(V_compression,P_compression,'linewidth',2,'color', 'b');
plot(V_expansion,P_expansion,'linewidth',2,'color', 'b');
plot([V2 V3],[P_2 P_3],'linewidth',2,'color', 'r');
plot([V4 V5],[P_4 P_5],'linewidth',2,'color', 'b');
plot([V5 V2],[P_5 P_0],'linewidth',2,'color', 'b');
plot(V2,P_2,'*','color','r');
plot(V3,P_3,'*','color','r');
xlabel('Volume (m^3)');
ylabel('Pressure (Pa)');
%% Logarithmic Plot of P-V Diagram 
figure
sgtitle('Logarithmic Plot of P-V Diagram');
loglog([V2,V1,V2,V3,V4,V5,V2],[P_0,P_1,P_2,P_3,P_4,P_5,P_0]);
xlim([5*10^-5 1*10^-3]);
ylim([0.5*10^5 3.5*10^6]);
xlabel('LOG V (m^3)');
ylabel('LOG P (Pa)');
grid on; 
%% Parameters for plotting
P_compression_max=P_compression(180);
P_combustion_min=P_expansion(1);
Theta_compression= linspace(180,360,180);
Theta_expansion = linspace(360,540,180);
Theta_exhaust=linspace(540,720,180);
Theta_full_cycle= linspace(0,720,720);
Pressure_exhaust=linspace(P_4,P_5,180);
Pressure_combustion=linspace(P_compression_max, P_combustion_min,180);
Theta_combustion=linspace(Theta_compression(180),Theta_expansion(1),180);
%% Pressure Change During Full Engine Cycle Plot 
figure
sgtitle('Pressure Change During Full Engine Cycle')
plot(Theta_Intake,P_Intake,Theta_compression,P_compression,Theta_combustion,Pressure_combustion,Theta_expansion,P_expansion,Theta_exhaust,Pressure_exhaust,'lineWidth',2)
legend('Intake Stroke','Compression Stroke','Combustion','Power Stroke','Exhaust Stroke')
xlim([0 730]) 
grid on 
xlabel('Theta (degrees)')
ylabel('Pressure (Pa)')
%% Flow velocity plot
figure
sgtitle('Flow velocity during intake')
plot(Theta_Intake,Flow_velocity)
grid on 
xlabel('Theta (degrees)')
ylabel('Velocity (m/s)')
%% Mass added plot
figure
sgtitle('Mass flow during intake')
plot(Theta_Intake,Mass_added_simulink)
grid on 
xlabel('Theta (degrees)')
ylabel('Mass (kg)')
%% 
figure
Pressure_Pico_Pa=6894.757*Pressure_Pico;   %% BAR
Theta_Pico = 360/70.4315*t_Pico ;     %% time to Theta 
plot(Theta_Pico,Pressure_Pico_Pa)
xlim([0 730 ])
hold on 
% plot(Theta_Intake,P_Intake,Theta_compression,P_compression,Theta_combustion,Pressure_combustion,Theta_expansion,P_expansion,Theta_exhaust,Pressure_exhaust,'lineWidth',2)
legend('Experimental Pressure Data','Simulated Model Pressure Data ','Location','Northwest')
xlabel('Crank Angle(degrees)')
ylabel('Pressure Pa')
hold off 
%% Engine size (Liters)
Engine_size = round(V_cyl*4*1000);                                            %Bore * Number of cylinders * 1000
fprintf(' The Size of this Engine is %d Liters \n',Engine_size)
%% Otto efficiency
Efficiency = (1- ((T4-T1)/(T3-T2)))*100;                                      % Calculate engine efficency 
fprintf('The efficiency of this Engine is %d \n',Efficiency)                  % Result in Command Window
%% Work output
cv=R/(gma-1);
Qin = m_end_comp*cv*(T3-T2);                                                   %Heat addition
Qout =m_end_exp *cv*(T4-T1);                                                   % Heat rejection
work= Qin-Qout;                                                                % Work output
fprintf(' The Work Done per Engine Cylinder is %d J \n',work)                  % Result in Command Window
%% Positive and Negative Work from Area Under P-V Curve  
trapz(V_compression,P_compression);                                            % Area under compression (negative)
trapz(V_expansion,P_expansion);                                                % Area under expansion (positive)
Work = trapz(V_expansion,P_expansion)+trapz(V_compression,P_compression);      % Area under p-V diagram = WORK DONE BY ONE ENGINE CYCLE
fprintf(' The Alternative of Work Calculation by integrating the area under the P-V curve per Engine Cycle is %d J \n',Work)
%% Power Output of Engine in J/s  
power      = work*25;                                                          % Work by cycles per second is power J/s 
fprintf('The Power Ouput of the Engine is %d J/s \n',power)                    % Result in Command Window
%% Power Ouput of Engine In HP 
HP         = power/745.69992140323;                                            % Converts power to horsepower
fprintf('The Power of the Engine is  %d HP \n' ,HP)                            % Result in Command Window
%% Torque in N.m
Torque     = power*30/(pi*engine_speed);                                       % Coverts power to torque
fprintf(' The Engine Produces  %d N.m of Torque \n',Torque)                    % Result in Command Window
%% Mean Effective Pressure
MEP= work/(V1-V2);
fprintf(' The Mean Effective Pressure inside the Engine Cylinder is %d Pa \n',MEP)
%% Maximum Flow velocity 
Max_Flow_velocity = max(Flow_velocity);
fprintf(' The Maximum Flow velocity of fluid is %d m/s \n',Max_Flow_velocity)
%% Function for Dynamic Compression & Expansion
function [V] = piston_kinematics(Bore,Stroke,ConRod,CR,start_crank_angle,end_crank_angle)
 s=Stroke;
 B=Bore;
 Cr=CR;
 a=Stroke/2;
 r=ConRod;
 Theta= linspace(start_crank_angle*pi/180,end_crank_angle*pi/180,180);
 V=0.5*s*(pi*B.^2/4)*(2/(Cr-1)+1-cos(Theta)+a/(4*r)*(1-cos(2*Theta))); % same volume calculation as in Simulinkmodel
end