function dislivello = crea_dislivello(len,y)
    %Function to create the descent pattern from a random variable,
    %deciding the length and the steepness of descent

    %initialises the vector to zero
    dislivello=zeros(1,len);
    
    %Uses the random variable to decide at what point of the driving cycle
    %to start the descent
    start=ceil(len*y);
    i=start;
    
    %The descent is from 0 to -10%, depending on the random variable
    while (i<len)
        dislivello(i)=-y*5.71*pi/180;
        i=i+1;
    end
end