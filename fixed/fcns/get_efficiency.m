function eta = get_efficiency(N, input_body)


eta_tot = 0.95^2;

switch input_body
    case 'sun'
        R = 1/N;
%         Z_tot = 1-N;
        eta = R + (1-R)*eta_tot;
    case 'arm'
        R = 1/N;
        Z_tot = 1-R;
        if R>0
            eta = (Z_tot - 1)/(Z_tot*eta_tot - 1);
        else
            eta = (Z_tot - 1)/(Z_tot/eta_tot - 1);
        end
        
    otherwise
        disp('input to the gear should be either sun / inv-sun')
end
