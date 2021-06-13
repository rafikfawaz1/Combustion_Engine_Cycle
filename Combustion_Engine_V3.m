%% Workspace clear
clear all;
clc;
close all;

%% Load Engine Data from real Experiment In-cylinder Compression Measurement (PicoScope Data)
% Data=xlsread('Toyota_RAV 4_2013_Petrol_20210610-0001_1.csv');
% t_Pico=Data(:,1);           %% All Time Data 
% Pressure_Pico = Data(:,4);  %% All Pressure data 
% plot(t_Pico,Pressure_Pico)  %% Plot

%% Parameters 
warning('off','all')
steps=1000;                     % calculation-steps per engine revolution.
engine_speed=3000;              % rpm
stepsize=(60/engine_speed)/steps;
sim_time=stepsize*(2*steps-1) ;  % two engine revolutions, one cycle
R      = 287;                   % Specific Universal gas constant(J/kg.K)
Rho    = 1.27;                  % Density of air (kg/m^3)
gma    = 1.35;                  % Specific heat ratio
MAP    = 80000;                 % Manifold Absolute Pressure [Pa]
P_ambient = 100000;             % Ambient pressure [Pa
CR     = 10;                    % Compression ratio  
Bore   = 82.7*10^-3;            %[m]
Stroke = 93*10^-3;              %[m]
V_cyl  =(Bore/2)^2*pi*Stroke;   % cylindre volume
V_comp =V_cyl/(CR-1);           % Compression ratio
CrankRadius=Stroke/2;           
ConRod =140*10^-3;              % Length of conrod
Valve_lift=9*10^-3;
Nr_of_valves=2;
Valve_dia=30*10^-3;
Cd=0.35;    % discharge coefficient of flow over intake valve
P_0=100000;  % Pa,  atmosferic, at start of intake stroke
T_air=323;  %air temperature at start of intake stroke
m_0=P_0*(V_comp)/(R*T_air);  %mass of air inside cylinder, at start of intake
V_sonic = 20.0457*sqrt(T_air); % Sonic velocity (m/s)
LHV=44400*10^3;  %%energy content(J/kg)
AFR =14.7;       % actual Air Fuel ratio,  [kg/kg]
AFR_stoich=14.7; % Stoichiometric air fuel ratio for the applied fuel.
V1=V_comp+V_cyl; % Total volume of cylinder
V2=V_comp;       % Compression Volume of cylinder 
P_Start=m_0*R*T_air/V2;   % Pressure at the start of intake stroke 
V3=V2;  % combustion  % isochoric process
Theta_0to180=linspace(0 , 180 , 500);

%% Simulate model
sim('Intake_2020') % Simulation of model to get initial pressure values  
P_1=max(P_Intake);    % Pressur at the start of Compression
P_2=max(P_end_compression);   % pressure end of compression
P_3=max(P_end_combustion);  % Pressur at the end of Combustion
C2=P_1*max(V_Intake).^gma;      % Isometric compression
C3=P_3*V3.^gma;      % Isometric expansion
%% Added mass
Mass_added=max(Mass_added_simulink);
%% Starting Point
% Starting of Intake process 
P_Start=P_Intake(1);
V3=V2;
%% Point 1 
% End of Intake
T1=T_air;  %% from PDF minor project (assumption)
%% point 2 
% End of Compression
% Starting of Combustion
V3=V2 ;  
T2=nthroot(((T_air^(gma/(gma-1)))*(V1/V2)^gma),(gma/(gma-1))); %% from PDF minor project
m_end_comp= (P_2*V2)/(R*T2);
%% Point 3 
% End of Combustion
% Strating of Expansion
T3=T2*(P_3/P_2);
%% point 4 
% End of Expansion
% Start of Exhaust
V4=V1; 
P_4=P_3*(V3/V4)^gma;     %Pressure at the end of Expansion
T4=nthroot(((T3^(gma/(gma-1)))*(V2/V1)^gma),(gma/(gma-1)));
m_end_exp= (P_4*V4)/(R*T4);
%% Exhaust
V5=V_comp;
P_5=P_4; %Exhaust is at constant pressure

%% Point Plots

figure
sgtitle('p-V points of engine cycle')
plot(V2,P_0,'*','color','r')
hold on
plot(V1,P_1,'*','color','r')

plot(V2,P_2,'*','color','r')

plot(V3,P_3,'*','color','r')
plot(V4,P_4,'*','color','r')
plot(V5,P_5,'*','color','r')

xlabel('Volume (m^3)')
ylabel('Pressure (Pa)')
hold off 
%% Dynamic Compression 
V_compression = piston_kinematics(Bore,Stroke,ConRod,CR,180,360);
P_compression=C2./V_compression.^gma;
%% Dynamic Expansion
C3=P_3*V3.^gma;
V_expansion= piston_kinematics(Bore,Stroke,ConRod,CR,360,540);
P_expansion=C3./V_expansion.^gma;
%% p-V diagram Plot
figure
sgtitle('P-V diagram')
hold on 
plot(V_Intake,P_Intake,'linewidth',2,'color', 'b')
plot(V_compression,P_compression,'linewidth',2,'color', 'b')
 plot(V_expansion,P_expansion,'linewidth',2,'color', 'b' )
 plot([V2 V3],[P_2 P_3],'linewidth',2,'color', 'r')
 plot([V4 V5],[P_4 P_5],'linewidth',2,'color', 'b')
 plot([V5 V2],[P_5 P_0],'linewidth',2,'color', 'b')
 plot(V2,P_2,'*','color','r')
 plot(V3,P_3,'*','color','r')
 xlabel('Volume (m^3)')
 ylabel('Pressure (Pa)')
 %% Logarithmic Plot of P-V Diagram 
figure
sgtitle('Logarithmic Plot of P-V Diagram ')
loglog([V2,V1,V2,V3,V4,V5,V2],[P_0,P_1,P_2,P_3,P_4,P_5,P_0])
xlim([5*10^-5 1*10^-3])
ylim([0.5*10^5 3.5*10^6])
 xlabel('LOG V (m^3)')
 ylabel('LOG P (Pa)')
 grid on 
 %% Params For Plotting
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
% %% Plot of Picoscope Data vs. Our data (in-cylinder Compression Measurement)
% figure
% Pressure_Pico_1=100000*Pressure_Pico;   %% Bar To PSI 
% Theta_Pico = 360/70.4315*t_Pico ;     %% time to Theta 
% plot(Theta_Pico,Pressure_Pico_1)
% xlim([0 730 ])
% hold on 
% plot(Theta_Intake,P_Intake,Theta_compression,P_compression,Theta_combustion,Pressure_combustion,Theta_expansion,P_expansion,Theta_exhaust,Pressure_exhaust,'lineWidth',2)
% legend('Experimental Pressure Data','Simulated Model Pressure Data ','Location','Northwest')
% xlabel('Crank Angle(degrees)')
% ylabel('Pressure PSI')
% hold off 
%% Flow velocity plot
figure
sgtitle('Flow velocity during intake')
plot(Theta_0to180,Flow_velocity)
grid on 
xlabel('Theta (degrees)')
ylabel('Velocity (m/s)')
%% Mass added plot
figure
sgtitle('Mass flow during intake')
plot(Theta_0to180,Mass_added_simulink)
grid on 
xlabel('Theta (degrees)')
ylabel('Mass (kg)')
%% Engine size (Liters)
Engine_size = round(V_cyl*4*1000);%%Bore * Number of cylinders * 1000
fprintf(' The Size of this Engine is %d Liters \n',Engine_size)
%% Otto efficiency
Efficiency = (1- ((T4-T1)/(T3-T2)))*100;
fprintf(' The efficiency of this Engine is %d percent \n ',Efficiency)

%% Work output
cv=R/(gma-1);

Qin = m_end_comp*cv*(T3-T2); %Heat addition
Qout =m_end_exp *cv*(T4-T1); % Heat rejection
work= Qin-Qout; % Work output
fprintf(' The Work Done per Engine Cylinder is %d J \n',work)
%% Positive and Negative Work from Area Under P-V Curve  
trapz(V_compression,P_compression);% area under compression (negative)
trapz(V_expansion,P_expansion);% area under expansion (positive)
Work= trapz(V_expansion,P_expansion)+trapz(V_compression,P_compression);% area under p-V diagram = WORK DONE BY ONE ENGINE CYCLE
fprintf(' The Alternative of Work Calculation by integrating the area under the P-V curve per Engine Cycle is %d J \n',Work)
%% Power Output of Engine in J/s  
power = work*25; %work by cycles per second is power J/s 
fprintf(' The Power Ouput per Engine Cylinder is %d J/s \n',power)
%% Power Ouput of Engine In HP 
HP=power/745.69992140323;
fprintf(' The Power output per Engine Cylinder is  %d HP \n' ,HP)
%% Torque in N.m
Torque= power*30/(pi*engine_speed);
fprintf(' The Engine Produces  %d N.m of Torque \n',Torque)
%% Mean Effective Pressure
MEP= work/(V1-V2);
fprintf(' The Mean Effective Pressure inside the Engine Cylinder is %d Pa \n',MEP)
%% Maximum Flow velocity 
Max_Flow_velocity = max(Flow_velocity);
fprintf(' The Maximum Flow velocity of fluid is %d m/s \n',Max_Flow_velocity)
%% Function for Dynamic Compression & Expansion
function [V] = piston_kinematics(Bore,Stroke,ConRod,CR,start_crank_angle,end_crank_angle);

 s=Stroke;
 B=Bore;
 Cr=CR;
 a=Stroke/2;
 r=ConRod;
 Theta= linspace(start_crank_angle*pi/180,end_crank_angle*pi/180,180);
 V=0.5*s*(pi*B.^2/4)*(2/(Cr-1)+1-cos(Theta)+a/(4*r)*(1-cos(2*Theta)));
end



