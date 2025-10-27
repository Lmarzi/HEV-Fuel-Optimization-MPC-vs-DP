function [toreturn,SOC,mf]= my_full_horizon(u,SOC_0,StateUpdate)
    %Function to simulate the StateUpdate function across the entire
    %horizon, given u,the intial state and the handle of the StateUpdate
    %funciton

    %Allocate all variables and intialise the first value of the state
    %variable to the initial variable
    N=length(u);
    SOC=zeros(1,N+1);
    SOC(1)=SOC_0;
    mf=zeros(1,N);
    mb=zeros(1,N);
    seq=zeros(1,N);
    Treq=zeros(1,N);
    Tgive=zeros(1,N);
    I_c=zeros(1,N);
    V_c=zeros(1,N);
    out= struct("Tm",zeros(N,1),"Tmmin",zeros(N,1),"Tmmax",zeros(N,1),"Te",zeros(N,1),"Temax",zeros(N,1));
    
    %Simulate across the entire horizon
    for i=2:N+1
        [mf(i-1),SOC(i),seq(i-1),Treq(i-1),Tgive(i-1),mb(i-1),I_c(i-1),V_c(i-1),out(i-1)]=StateUpdate(u(i-1),SOC(i-1),i-1);
    end

    %Return the cost function and the constraints
    mf_eq=sum(mf+mb.*seq);
    c=[-SOC(2:end)+0.7,SOC(2:end)-0.4,I_c,V_c,-([out.Tm]-[out.Tmmin])./[out.Tmmin],(-[out.Tm]+[out.Tmmax])./[out.Tmmax],[out.Te]./[out.Temax],(-[out.Te]+[out.Temax])./[out.Temax]];
    ceq=[];
    toreturn=[mf_eq,ceq,c]';
    SOC=SOC(2:end);
end