B=zeros(4016,48);
for i=1:4016
    for j=4:48
                   if mod(j,3)==1 
           B(i,j)=(florence(i,j)-florence(i,10))*1800/(florence(i,5)-florence(i,47));
                   elseif mod(j,3)==2
                   B(i,j)=(florence(i,j)-florence(i,11))*1800/(florence(i,5)-florence(i,47)); 
               else 
                   B(i,j)=(florence(i,j)-florence(i,12))*1800/(florence(i,5)-florence(i,47));
                   end
    end
end
                   
        
    