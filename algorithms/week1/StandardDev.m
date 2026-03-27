function [StandDev] = StandardDev(DataFromSenzor)

velikost = size(DataFromSenzor,1);
prumer = sum(DataFromSenzor,1)/velikost;




sq_diff = (DataFromSenzor - prumer) .^ 2;
variance = sum(sq_diff, 1) / (velikost - 1);
StandDev = sqrt(variance);

    
  
    
    
    % vyberovy rozptyl a nasledna odmocnina
    
    


end