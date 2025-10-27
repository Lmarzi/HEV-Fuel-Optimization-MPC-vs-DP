function [driving_cycle,name]=pick_cycle(x,y,descent_odds)
    if x<=0.33
       driving_cycle=load("Data_files\ARTEMIS.mat");
       driving_cycle=driving_cycle.ARTEMIS;
       name="ARTEMIS";
   elseif x<=0.66
       driving_cycle=load("Data_files\ARTEMIS_road.mat");
       driving_cycle=driving_cycle.ARTEMIS_road;
       name="ARTEMIS_road";
    else
       driving_cycle=load ("Data_files\WLTC.mat");
       driving_cycle=driving_cycle.WLTC;
       name="WLTC";
    end
    driving_cycle=[driving_cycle;zeros(1,length(driving_cycle))];
    if y>1-descent_odds
        driving_cycle(4,:)=crea_dislivello(length(driving_cycle),x);
    end
end