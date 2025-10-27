function [mf,SOC_new,seq,Treq,Tgiv,mb,I_c,V_c,out] = my_hev(speed,acceleration,gear,dislivello,cur_SOC,u,F1,F2,Tmmax_int,Tmmin_int,Temax_int,t,globconstr)

% VEHICLE PARAMETERS
wheel_radius = 0.281154; %m
vehicle_mass = 1200; %kg

% Coasting down test parameters
a_cd = 93.086154;  %N
b_cd = 2.5373490;  %N/(m/s))
c_cd = 0.38382359; %N/(m/s)^2)
g=9.81;


%The model is not fit for speed <= 0, so we just standardise the driving
%cycle
speed=max(speed,5/3.6);

inp.W{1}=speed;
inp.W{2}=acceleration;
inp.W{3}=gear;
inp.W{4}=dislivello;
inp.X{1}=cur_SOC;
inp.U{1}=u;

% Wheel speed (rad/s)
wv  = inp.W{1} / wheel_radius;

Tv = (vehicle_mass*g*sin(inp.W{4})+a_cd*cos(inp.W{4})+b_cd*inp.W{1}*cos(inp.W{4}) + c_cd*inp.W{1}^2 + vehicle_mass*inp.W{2})*(inp.W{1}>0) * wheel_radius;

% TRANSMISSION
gearbox_efficiency = 0.96;
% final drive ratio (from main axle to wheel)
t0 = 3.1725494;
% gear box (from motor to main axle) 1st to 5th gear
tprime = [3.417 1.958 1.276 0.943 0.757];
% gear ratios
r_gear = t0*tprime;
% Crankshaft speed (rad/s)
wg  = (inp.W{3}>0) * r_gear(inp.W{3} + (inp.W{3}==0)) * wv; 
% Crankshaft torque (Nm)
Tg  = (inp.W{3}>0) * (Tv>0)  * Tv / r_gear(inp.W{3} + (inp.W{3}==0)) / gearbox_efficiency...
    + (inp.W{3}>0) * (Tv<=0) * Tv / r_gear(inp.W{3} + (inp.W{3}==0)) * gearbox_efficiency;
Treq=Tg;

%Use iterpolation to extract the saturation values from the crankshaft speed
Tm_min=Tmmin_int(wg);
Tm_max=Tmmax_int(wg);
Te_max = Temax_int(wg);

% Total required torque (Nm)
Ttot = Tg;
% Torque provided by engine
Te = (Ttot>0)* (inp.U{1}<=1)  * (1-inp.U{1})*Ttot;
Tb  = (Ttot<=0) * (1-inp.U{1})*Ttot;
% Torque provided by electric motor
Tm = inp.U{1} * Ttot;
Tgiv=Te+Tb+Tm;

%Compute mf from the efficiency of the motor extracted trough interpolation
gasoline_lower_heating_value = 43.308*10^6; %J/kg
e_th = F2(wg*ones(size(Te)),Te);

mf=Te*wg/e_th/gasoline_lower_heating_value;

%SOC update

% Calculate electric power consumption
% motor efficiency map (indexed by speed list and torque list)

e=F1(abs(Tm),wg*ones(size(Tm)));
Pm =  (Tm<0) * wg*Tm*e + (Tm>=0) * wg*Tm/e;

% BATTERY SIMPLIFIED MODEL-Only a constant voltage generator
Vn = 100;
battery_capacity =45; %Ah 

%Set the new SOC value, the second line is necessary because Ib can have
%non zero immaginary part if a value of u outside of voltage constraints is
%evaluated, but it must return real values in any case
SOC_new = -1/(battery_capacity*3600)*Pm/Vn+inp.X{1};
SOC_new= (conj(SOC_new)+SOC_new)/2;

% Battery power consumption
% columbic efficiency (0.9 when charging)
% Battery current
r = 39e-3; %ohm
eff = (Pm>0) + (Pm<=0) * 0.9;
Ib  =   eff * (Vn-sqrt(Vn^2 - 4*r*Pm))/(2*r);
Ib   = (conj(Ib)+Ib)/2;

max_disch_curr = 100;   %A
max_char_curr    = 125; %A
Im = (Pm>0) * max_disch_curr + (Pm<=0) * max_char_curr;
I_c=(Pm>0)*(Im-Ib)/Im-(Pm<=0)*(Ib-Im)/Im;
V_c=(Vn^2 - 4*r*Pm)/Vn^2;

Pb =   Ib * Vn;
Pb   = (conj(Pb)+Pb)/2;
mb=Pb/gasoline_lower_heating_value;
seq = ((Tm>0)*1/(0.2757*0.8879)+(Tm<0)*0.8879/0.2757)*25*(-(2*(SOC_new-0.55))^3+(2*(0.68-0.55))^3)+(globconstr==1)*(t<1000)*1000*(0.55-SOC_new)*exp(-t/150);
out.Tmmax = Tm_max;
out.Tmmin = Tm_min;
out.Temax = Te_max;
out.Te = Te;
out.Tm = Tm;



end