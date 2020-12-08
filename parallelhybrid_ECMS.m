function [u]=parallelhybrid_ECMS(x)
%global data
%Given inputs from the driving cycle and controller this script should return
%the torques on the electric motor and combustion engine.
w_ice=x(1);
dw_ice=x(2);
T_req=x(3);
lambda=x(4);
% w_ice=0.1;
% dw_ice=1;
% T_req=1000;
% lambda=2.5;
%% Data of Wheel and Gearbox
H_l= 44.6e6; %J/kg
roha_l=732.2; %Kg/m3
roha_a=1.18; % kg/m3 
%% Engine Parameters
J_e=0.2; % kgm2
T_e_max= 115; %Nm
V_d=1.497e-3; % m3
%V_d=2.24/1000;    %m3
n_r= 2;
m_e=1.2;  % kg/KW
  
%% Willans approximation of engine efficiency
e=0.4;
p_me_0=0.1e6; %MPa
%% Battery Configuration
Q_0=6.5; % Ah
U_oc=300; %V
I_max= 200;%A
I_min=-200; %A
R_i= 0.65; % ohm
m_batt= 45; % kg
  
%% Motor and Generator
n_machine=0.9;
T_em_max=400; %Nm
P_em_max=50; % kW
m_em=1.5; %kg/Kw
  
%% Powertrain Power
P_pt_max=90.8; % kW
  
%% Model Parameters
g=9.81;
c_D=0.32;
c_R=0.015;
A_f=2.31; % m2
m= 1500; %kg
%m= (1500-120)+36.8; %kg
%m=1500-120;   %kg
r_w=0.3; %m
J_w=0.6; % kgm2
m_wheel=J_w/(r_w^2);
n_gearbox=0.98;

%% Range
N_e= 0:800:5000;
w_e= 300; % rad/s2

%% Data of Electric Machine, Battery and IC engine

I=linspace(I_min,I_max,50000);
Power_battery=(U_oc*I)-(R_i*I.^2);   % Power of Battery

Power_electric_machine=Power_battery.*0.9.^sign(I);  % Power of Electric Machine

T_em=(Power_battery.*n_machine.^sign(I))./w_ice;   % Torque of Electric Machine

T_ice=T_req-T_em;  % Torque of Engine

x=((p_me_0*V_d)/(4*pi));
y=J_e*dw_ice;

costVector=(w_ice/(e*H_l))*(T_ice+x+y); % Fuel Consumption

%% Hamiltonian
P_f=H_l*costVector;
P_f(costVector<0)=0;
P_ech=I*U_oc;

if w_ice==0
T_ice=0;
T_em=0;
u=[T_ice;T_em];
else
H=P_f+(lambda*P_ech);
H((T_ice+(J_e*dw_ice))>115)=inf;
H(T_em>400 | T_em<-400)=Inf;
H(Power_electric_machine<-50000 | Power_electric_machine>50000)=Inf;
[q,i]=min(H);
T_ice=T_ice(i);
T_em=T_em(i);
u=[T_ice;T_em];
end
   


