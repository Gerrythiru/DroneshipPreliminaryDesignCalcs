%Calculations for G-30J Drone Preliminary Design%

format long;
close all;
clear;
clc;

fprintf('Calculations for G-30J Drone Preliminary Design, Input Mission and Initial Aircraft Parameters in the Script.\n');
disp(' ')
disp(' ')
%% Input Mission Parameters
Range = 1.64042*(10^6); % In Feet
Cruise_Speed = 877.7559;% In ft/s
MaxMach = 0.9;
K1 = 0.28;
K2 = 0.87;
Approach_Speed = 236.293;
Stall_Speed = 180.596;

%% Input Aircraft Parameters
Spec_fuel_cons = 5.55555556*(10^-4); %lb/s-lb
Cdo_cruise = 0.022;
Oswald_coeff = 0.6;
Cl_max = 1.5;
AR = 8.1;
W_pl = 500; %pounds

%% Selection of Basic Geometric Parameters%%

Leadingedge_Sweep = 35;
Taper_ratio = 0.17;
Q = tand(Leadingedge_Sweep) - ((1-Taper_ratio)/(AR*(1+Taper_ratio)));
Quarterchord_sweep = atand(Q); 


fprintf('QuarterChord Sweep Angle is %.3f degrees.\n', Quarterchord_sweep);
disp(' ')

%% Determening T/W and W/S%%
Vstall = 180.596;
q_stall = 0.5*0.0023769*(180.596)^2;
W_S_takeoff = q_stall*Cl_max;

q_req = (0.5)*(0.00089068)*(Cruise_Speed)^2;
T_W = (K2/K1)*(((q_req*Cdo_cruise)/(W_S_takeoff*K2))+((W_S_takeoff*K2)/(q_req*pi*AR*Oswald_coeff)));

fprintf('Wing Loading is %.3f pounds per sqaure feet.\n', W_S_takeoff);
fprintf('Thrust to Weight Ratio is %.3f.\n', T_W);
disp(' ')

%% T/W vs W/S plot
W_S = 0:1.5:100; 
T_Wo = (K2/K1)*(((q_req*Cdo_cruise)/(K2)./W_S)+((K2)/(q_req*pi*AR*Oswald_coeff).*W_S));

figure(1)
plot(W_S,T_Wo,'linewidth',2);
hold on
xline(W_S_takeoff,'linewidth',2);
plot(55,1,'d');

xlabel('W/S','fontsize',14)
ylabel('T/W','fontsize',14)
set(gca,'fontsize',13)
title('T/W vs W/S')
grid on

%% Calc L/D for Cruise
W_Scr = K2*W_S_takeoff;
L_D = 1/(((q_req*Cdo_cruise)/(W_Scr))+((W_Scr)/(q_req*pi*AR*Oswald_coeff)));

fprintf('Calculated Lift to Drag ratio is %.3f.\n', L_D);
disp(' ')

%% Weight Fraction%%

Wattack_Wcruisestart = exp(-(Range*Spec_fuel_cons)/(Cruise_Speed*L_D));
fprintf('Weight fraction for the 500km cruise is %.3f.\n',Wattack_Wcruisestart);

Mission_fuel_fraction = 1.06*(1-(Wattack_Wcruisestart));
fprintf('Mission fuel weight fraction is %.3f.\n',Mission_fuel_fraction);
disp(' ')

%% Log-log curve fit for empty weight expression%%
figure(2)
x = [5.4072; 8.7796; 9.9134; 6.8395; 3.4012; 7.6962; 9.3588; 7.2855; 7.9516; 7.4265]; % X values are ln(Wo)%
y = [-0.7437; -0.9555; -1.1494; -0.3174; -0.2877; -0.8398; -0.6798; -0.3347; -0.3507; -0.4605]; % Y values are ln(We/Wo)%
scatter(x,y);
xlabel('Ln(Wo)','fontsize',14)
ylabel('Ln(We/Wo)','fontsize',14)
set(gca,'fontsize',13)
title('Ln(We/Wo) vs Ln(Wo)')
grid on

%%Iteration Solving for Wo
Ke1 = -0.09493;
Ke2 = 0.09115;
W_fo = Mission_fuel_fraction;
tol = 1;
Wo_in = 2000; % Initial guess for weight
A_1 = W_pl;
A_2 = 1 - W_fo;
A_3 = -exp(Ke2);
delta_W = 2;
while delta_W >= tol
    Wo_out = (A_1)/(A_2 + A_3*(Wo_in)^(Ke1));
    delta_W = abs(Wo_out - Wo_in);
    Wo_in = Wo_out;
    fprintf(' \n Takeoff Weight is %f', Wo_out)
    disp(' lbs')
end
disp(' ')
disp(' ')

%% Estimating Wing Area and Wing Geometry%%
Takeoff_Weight = 1680.350639; %%Wo from Iteration%%
Tot_Thrust = Takeoff_Weight*T_W;
fprintf('Total Thrust required is %.3f pounds.\n', Tot_Thrust);
disp(' ')

%All values in feet%
Wing_area = (1680.350639)/(W_S_takeoff);
fprintf('Calculated Wing Area from Wing Loading is %.3f sqaure feet.\n', Wing_area);

Wing_Span = (Wing_area*AR)^0.5; %AR is 8.1!
fprintf('Calculated Wing Span is %.3f feet.\n', Wing_Span);

Vtail_rootchord = (2*Wing_area)/(Wing_Span*(1 + Taper_ratio));
Vtail_tipchord = Taper_ratio*Vtail_rootchord;
fprintf('Root and Tip chord length of the wing is %.4f feet and %.4f feet respectively.\n', Vtail_rootchord, Vtail_tipchord);

MAC = (2/3)*(Vtail_rootchord)*((1+Taper_ratio+(Taper_ratio)^2)/(1+Taper_ratio));
Y_bar = (Wing_Span/6)*((1+2*Taper_ratio)/(1+Taper_ratio));

fprintf('Length of Mean Aerodynamic chord (MAC) is %.4f feet, while the spanwise location (Ybar) of MAC is %.4f feet.\n', MAC, Y_bar);

Flap_span = 2/3*(0.5*Wing_Span);
Flap_Wingspan_ratio = Flap_span/Wing_Span;
fprintf('Flap/WingSpan ratio is %.3f, using this value we assume FlapChord/WingChord as 0.30 from historical data.\n', Flap_Wingspan_ratio);
disp(' ')

%% Fuselage Geometry
a = 0.67; %From Raymer's Corelation Coefficients
C = 0.43;
Fuse_L = a*(Takeoff_Weight)^C;

SlendernessRatio = 9;
FuseMaxD = Fuse_L/SlendernessRatio;
fprintf('Length of the Fuselage is %.3f feet, and selecting Slenderness ratio as 9, Fuselage Max Diameter is %.3f feet.\n', Fuse_L, FuseMaxD);

%Here we will assume tail lengths for both vertical and horizontal tails as
%percentage of total fuselage length
TailLength = 0.515*Fuse_L; %Taken as 51.5% of Fuselage length
fprintf('Assumed Tail length for both vertical and horizontal tails is %.3f feet.\n', TailLength);

%% Tail Geometry
cHT = 1;
cVT = 0.09;
Htail_area = (cHT*MAC*Wing_area)/TailLength;
fprintf('Horizontal Tail Area is %.3f sqaure feet.\n', Htail_area);
Htail_Span = (Htail_area*4.6)^0.5; % Selected AR is 4.6!
fprintf('Horizontal Tail Span is %.3f feet.\n', Htail_Span);
Htail_rootchord = (2*Htail_area)/(Htail_Span*(1 + 0.4)); %Selected Taper ratio is 0.4
Htail_tipchord = 0.4*Htail_rootchord;
fprintf('Root and Tip chord length of the Horizontal Tail is %.4f feet and %.4f feet respectively.\n', Htail_rootchord, Htail_tipchord);

Ce_C = 0.25;
Htail_AvgChord = Htail_Span/4.6;
ElevatorChord = Ce_C*Htail_AvgChord;
fprintf('Elevator Chord Length is %.3f feet.\n', ElevatorChord);
disp(' ')


Vtail_area = (cVT*Wing_Span*Wing_area)/TailLength;
fprintf('Vertical Tail Area is %.3f sqaure feet.\n', Vtail_area);
Vtail_Span = (Vtail_area*1.7)^0.5; % Selected AR is 1.7!
fprintf('Vertical Tail Span is %.3f feet.\n', Vtail_Span);
Vtail_rootchord = (2*Vtail_area)/(Vtail_Span*(1 + 0.4)); %Selected Taper ratio is 0.4
Vtail_tipchord = 0.4*Vtail_rootchord;
fprintf('Root and Tip chord length of the Vertical Tail is %.4f feet and %.4f feet respectively.\n', Vtail_rootchord, Vtail_tipchord);
Cr_C = 0.32;
Vtail_AvgChord = Vtail_Span/1.7;
RudderChord = Cr_C*Vtail_AvgChord;
fprintf('Rudder Chord Length is %.3f feet.\n', RudderChord);
disp(' ')

%%Engine Diameter and Length
Engine_D = 1.9685; % Assumption In feet
Engine_L = 4*Engine_D; %Assumption that Length is 4xDiameter
fprintf('The Engine Diameter is %.4f feet and its Length is %.4f feet.\n', Engine_D, Engine_L);










 










































