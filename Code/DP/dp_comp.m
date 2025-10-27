function [res] = dp_comp(drive_cycle,SOC_START,final_SOC)

N=length(drive_cycle(1,:));
speed_vector=drive_cycle(1,1:N);
acceleration_vector=drive_cycle(2,1:N);
gearnumber_vector=drive_cycle(3,1:N);
for i=1:N
    if(speed_vector(i)<=5/3.6)
        speed_vector(i)=5/3.6;
    end
end

road_slope = drive_cycle(4,1:N);

%SOC constraints
SOC_sup = 0.7;
SOC_inf = 0.4;
SOC_cons = SOC_START;

% create grid
clear grd
Path = 0.001;
Nx = floor((SOC_sup-SOC_inf)/Path+1);
grd.Nx{1}    = Nx; 
grd.Xn{1}.hi = SOC_sup; 
grd.Xn{1}.lo = SOC_inf;
% set initial condition on the state
grd.X0{1} = SOC_cons;

% final state constraints
grd.XN{1}.hi = final_SOC;%SOC_cons+0.01;
grd.XN{1}.lo = final_SOC-1e-10;%SOC_cons;

Inp_max = 1;
Inp_min = -1;
Nu = floor((Inp_max-Inp_min)/0.01+1);
%Input 
grd.Nu{1}    = Nu; 
grd.Un{1}.hi = Inp_max; 
grd.Un{1}.lo = Inp_min;	% Att: Lower bound may vary with engine size.

% define problem
clear prb
%input sequence
prb.W{1} = speed_vector;
prb.W{2} = acceleration_vector; 
prb.W{3} = gearnumber_vector; 
prb.W{4}=  road_slope;
%Sampling time definition for discretization
prb.Ts = 1;
prb.N  = N/prb.Ts;

% set options
options = dpm();
options.SaveMap=1;
options.MyInf = 1000;
options.BoundaryMethod = 'Line'; % also possible: 'none' or 'LevelSet';
if strcmp(options.BoundaryMethod,'Line') 
    %these options are only needed if 'Line' is used
    options.Iter = 5;
    options.Tol = 1e-8;
    options.FixedGrid = 0;
end
[res, dyn] = dpm(@hev,[],grd,prb,options);