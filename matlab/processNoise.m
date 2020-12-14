function [ sigma_a,sigma_alpha,z ] = processNoise( sequence,z )

if strcmp(sequence,'04')
    
    for i=1:size(z,2)
        
        %constant
        if z(5,i)<-.001
            z(5,i)=-.001;
        end
        if z(5,i)>.001
            z(5,i)=.001;
        end
        sigma_alpha_y(i) = 2e-6;
        
        if z(1,i)<-.001
            z(1,i)=-.001;
        end
        if z(1,i)>.001
            z(1,i)=.001;
        end
        
        sigma_a_x(i) = 7e-3;
        
        sigma_a_z(i) = 7e-5;
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-6;
        sigma_alpha_z(i) = 2e-6;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-5;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
        
    end
    
end



if strcmp(sequence,'06')
    
    for i=1:size(z,2)
        
        if (242<=i && i<366) || (449<=i && i<533)...
                ||(667<=i && i<756) ||(889<=i && i<968)
            
            sigma_alpha_y(i) = 2e-3;
        else
            %constant
            if z(5,i)<-.001
                z(5,i)=-.001;
            end
            if z(5,i)>.001
                z(5,i)=.001;
            end
            sigma_alpha_y(i) = 2e-6;
        end
        
        
        if (263<=i && i<350) || (665<=i && i<750)
            
            sigma_a_x(i) = 2e-4;
        else
            %constant
            sigma_a_x(i) = 7e-5;
        end
        
        
        sigma_a_z(i) = 2e-4;
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-6;
        sigma_alpha_z(i) = 2e-6;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-5;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
    end
    
end



if strcmp(sequence,'10')
    
    for i=1:size(z,2)
        
        if (1<=i && i<75) || (120<=i && i<276) || (352<=i && i<485)...
                || (527<=i && i<758) || (829<=i && i<9300)
            
            sigma_alpha_y(i) = 2e-3;
        else
            %constant
            if z(5,i)<-.001
                z(5,i)=-.001;
            end
            if z(5,i)>.001
                z(5,i)=.001;
            end
            sigma_alpha_y(i) = 2e-6;
        end
        
        if (1<=i && i<89) || (517<=i && i<687) || (825<=i && i<919)
            
            sigma_a_x(i) = 2e-4;
        else
            %constant
            sigma_a_x(i) = 7e-5;
        end
        
        
        if z(3,i)<-1.5
            z(3,i)=-1.5;
        end
        sigma_a_z(i) = 2e-5;
        
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-6;
        sigma_alpha_z(i) = 2e-6;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-5;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
    end
    
    
end


if strcmp(sequence,'07')
    
    for i=1:size(z,2)
        
        if  (1<=i && i<63) || (106<=i && i<176) || (288<=i && i<367)...
                || (434<=i && i<502) || (526<=i && i<671)...
                || (712<=i && i<794) || (866<=i && i<948)
            
            
            sigma_alpha_y(i) = 2e-3;
        else
            %constant
            sigma_alpha_y(i) = 2e-6;
        end
        
        if (1<=i && i<62) || (107<=i && i<172) || (286<=i && i<371)...
                || (438<=i && i<583) || (720<=i && i<792)...
                || (878<=i && i<953)
            
            sigma_a_x(i) = 7e-4;
        else
            
            %constant
            sigma_a_x(i) = 7e-6;
        end
        
        
        if z(3,i)<-1.25
            z(3,i)=-1.25;
        end
        
        sigma_a_z(i) = 7e-5;
        
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-6;
        sigma_alpha_z(i) = 2e-6;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-5;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
    end
end


if strcmp(sequence,'08')
    
   for i=1:size(z,2)
    
    if  (23<=i && i<97) || (380<=i && i<449) || (464<=i && i<540)...
            || (677<=i && i<746) || (765<=i && i<886)...
            || (972<=i && i<1041) || (1055<=i && i<1117)...
            || (1285<=i && i<1346) || (1573<=i && i<1662)...
            || (1809<=i && i<1879) || (1898<=i && i<1970)...
            || (2134<=i && i<2200) || (2458<=i && i<2552)...
            || (2601<=i && i<2671) || (2839<=i && i<2975)...
           || (3004<=i && i<3067) || (3350<=i && i<3463) || (3727<=i && i<3786)...
            
        
        
        sigma_alpha_y(i) = 2e-3;
    else
        
        if z(5,i)<-.001
            z(5,i)=-.001;
        end
        if z(5,i)>.001
            z(5,i)=.001;
        end
        
        %constant
        sigma_alpha_y(i) = 2e-3;
    end
    
    if (33<=i && i<103) || (384<=i && i<440) || (468<=i && i<517)...
            || (687<=i && i<735) || (770<=i && i<832)...
            || (973<=i && i<1036) || (1059<=i && i<1116)...
            || (1284<=i && i<1352)  || (1570<=i && i<1646)...
            || (1805<=i && i<1859) || (1893<=i && i<1974)...
            || (2127<=i && i<2227) || (2460<=i && i<2548)...
            || (2611<=i && i<2674) || (2824<=i && i<2976)...
            || (3024<=i && i<3106) || (3371<=i && i<3469)...
            || (3721<=i && i<3795)
        
        
        sigma_a_x(i) = 2e-4;
    else
        
        %constant
        sigma_a_x(i) = 2e-6;
    end
    
    
        if z(3,i)<-1.35
            z(3,i)=-1.35;
        end
        sigma_a_z(i) = 2e-4;
    
    % standar deviation for angular acceleration noise
    sigma_alpha_x(i) = 2e-6;
    sigma_alpha_z(i) = 2e-6;
    
    % standar deviation for linear acceleration noise
    sigma_a_y(i) = 7e-5;
    
    sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
    sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
end


end



if strcmp(sequence,'00')
    
    for i=1:size(z,2)
        
        if  (84<=i && i<145) || (183<=i && i<230) || (397<=i && i<451)...
                || (558<=i && i<611) || (715<=i && i<773)...
                || (931<=i && i<982) || (1107<=i && i<1157)...
                || (1212<=i && i<1294) || (1360<=i && i<1436)...
                || (1538<=i && i<1578) || (1780<=i && i<1811)...
                || (1930<=i && i<1986) || (2102<=i && i<2140)...
                || (2311<=i && i<2352) || (2407<=i && i<2464)...
                || (2540<=i && i<2721) || (2774<=i && i<2906)...
                || (2961<=i && i<3035) || (3082<=i && i<3132) ...
                || (3257<=i && i<3314) || (3351<=i && i<3455)...
                || (3522<=i && i<3571) || (3658<=i && i<3705)...
                || (3904<=i && i<3998) || (4345<=i && i<4397)...
                || (4415<=i && i<4471)
            
            sigma_alpha_y(i) = 2e-3;
        else
            
            if z(5,i)<-.001
                z(5,i)=-.001;
            end
            if z(5,i)>.001
                z(5,i)=.001;
            end
            
            %constant
            sigma_alpha_y(i) = 2e-4;
        end
        
        if (89<=i && i<137) || (184<=i && i<231) || (398<=i && i<462)...
                || (562<=i && i<607) || (717<=i && i<774)...
                || (930<=i && i<972) || (1100<=i && i<1163)...
                || (1211<=i && i<1301)  || (1374<=i && i<1435)...
                || (1538<=i && i<1585) || (1780<=i && i<1811)...
                || (1919<=i && i<1975) || (2095<=i && i<2148)...
                || (2322<=i && i<2348) || (2414<=i && i<2459)...
                || (2673<=i && i<2718) || (2848<=i && i<2891)...
                || (2972<=i && i<3019) || (3089<=i && i<3151)...
                || (3255<=i && i<3323)|| (3364<=i && i<3465)...
                || (3515<=i && i<3575) || (3653<=i && i<3719)...
                || (3907<=i && i<4000) || (3903<=i && i<3991)...
                || (4350<=i && i<4469)
            
            
            sigma_a_x(i) = 2e-4;
        else
            
            %constant
            sigma_a_x(i) = 2e-5;
        end
        
        
        if 1<=i
            if z(3,i)<-1.5
                z(3,i)=-1.5;
            end
            
            sigma_a_z(i) = 2e-4;
        else %constant
            sigma_a_z(i) = 2e-5;
        end
        
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-6;
        sigma_alpha_z(i) = 2e-6;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-5;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
    end
    
end


% if strcmp(sequence,'15')
%     
%     for i=1:size(z,2)
%         
%         %constant
% %         if z(5,i)<-.001
% %             z(5,i)=-.001;
% %         end
% %         if z(5,i)>.001
% %             z(5,i)=.001;
% %         end
%         sigma_alpha_y(i) = 2e-1;
%         
% %         if z(1,i)<-.001
% %             z(1,i)=-.001;
% %         end
% %         if z(1,i)>.001
% %             z(1,i)=.001;
% %         end
%         
%         sigma_a_x(i) = 7e-1;
%         
%         sigma_a_z(i) = 7e-1;
%         
%         % standar deviation for angular acceleration noise
%         sigma_alpha_x(i) = 2e-1;
%         sigma_alpha_z(i) = 2e-1;
%         % standar deviation for linear acceleration noise
%         sigma_a_y(i) = 7e-1;
%         
%         sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
%         sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
%         
%     end
%     
% end


if strcmp(sequence,'15')
    
    for i=1:size(z,2)
        
        if (204<=i && i<301) || (430<=i && i<497)  || (517<=i && i<565)...
                ||(654<=i && i<707) ||(767<=i && i<813)...
            ||(908<=i && i<965) ||(1021<=i && i<1088)...
            ||(1141<=i && i<1200) ||(1397<=i && i<1459)...
            ||(1587<=i && i<1655) ||(1837<=i && i<1885)
        
            sigma_alpha_y(i) = 2e-3;
        else
            %constant
            if z(5,i)<-.001
                z(5,i)=-.001;
            end
            if z(5,i)>.001
                z(5,i)=.001;
            end
            sigma_alpha_y(i) = 2e-6;
        end
        
        
        if (204<=i && i<301) || (430<=i && i<497)  || (517<=i && i<565)...
                ||(654<=i && i<707) ||(767<=i && i<813)...
            ||(908<=i && i<965) ||(1021<=i && i<1088)...
            ||(1141<=i && i<1200) ||(1397<=i && i<1459)...
            ||(1587<=i && i<1655) ||(1837<=i && i<1885)
            
            sigma_a_x(i) = 2e-4;
        else
            %constant
            sigma_a_x(i) = 7e-5;
        end
        
        
        sigma_a_z(i) = 2e-4;
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-6;
        sigma_alpha_z(i) = 2e-6;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-5;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
    end
    
end



if strcmp(sequence,'14')
    
    for i=1:size(z,2)
        
        %constant
%         if z(5,i)<-.001
%             z(5,i)=-.001;
%         end
%         if z(5,i)>.001
%             z(5,i)=.001;
%         end
        sigma_alpha_y(i) = 2e-1;
        
%         if z(1,i)<-.001
%             z(1,i)=-.001;
%         end
%         if z(1,i)>.001
%             z(1,i)=.001;
%         end
        
        sigma_a_x(i) = 7e-1;
        
        sigma_a_z(i) = 7e-1;
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-1;
        sigma_alpha_z(i) = 2e-1;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-1;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
        
    end
    
end




% if strcmp(sequence,'14')
%     
%     for i=1:size(z,2)
%         
%         if (152<=i && i<200) || (298<=i && i<342)...
%                 || (365<=i && i<410) ||(500<=i && i<542) 
%         
%             sigma_alpha_y(i) = 2e-3;
%         else
%             %constant
%             if z(5,i)<-.001
%                 z(5,i)=-.001;
%             end
%             if z(5,i)>.001
%                 z(5,i)=.001;
%             end
%             sigma_alpha_y(i) = 2e-6;
%         end
%         
%         
%         if (152<=i && i<200) || (298<=i && i<342)...
%                 || (365<=i && i<410) ||(500<=i && i<542) 
%             
%             sigma_a_x(i) = 2e-4;
%         else
%             %constant
%             sigma_a_x(i) = 7e-5;
%         end
%         
%         
%         sigma_a_z(i) = 2e-4;
%         
%         % standar deviation for angular acceleration noise
%         sigma_alpha_x(i) = 2e-6;
%         sigma_alpha_z(i) = 2e-6;
%         % standar deviation for linear acceleration noise
%         sigma_a_y(i) = 7e-5;
%         
%         sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
%         sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
%     end
%     
% end
% 
% 


if strcmp(sequence,'13')
    
    for i=1:size(z,2)
        
        %constant
%         if z(5,i)<-.001
%             z(5,i)=-.001;
%         end
%         if z(5,i)>.001
%             z(5,i)=.001;
%         end
        sigma_alpha_y(i) = 2e-1;
        
%         if z(1,i)<-.001
%             z(1,i)=-.001;
%         end
%         if z(1,i)>.001
%             z(1,i)=.001;
%         end
        
        sigma_a_x(i) = 7e-1;
        
        sigma_a_z(i) = 7e-1;
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-1;
        sigma_alpha_z(i) = 2e-1;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-1;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
        
    end
    
end




if strcmp(sequence,'12')
    
    for i=1:size(z,2)
        
        %constant
%         if z(5,i)<-.001
%             z(5,i)=-.001;
%         end
%         if z(5,i)>.001
%             z(5,i)=.001;
%         end
        sigma_alpha_y(i) = 7e-3;
        
%         if z(1,i)<-.001
%             z(1,i)=-.001;
%         end
%         if z(1,i)>.001
%             z(1,i)=.001;
%         end
        
        sigma_a_x(i) = 7e-4;
        
        sigma_a_z(i) = 7e-4;
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-6;
        sigma_alpha_z(i) = 2e-6;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-5;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
        
    end
    
end



% 
% if strcmp(sequence,'11')
%     
%     for i=1:size(z,2)
%         
%         %constant
% %         if z(5,i)<-.001
% %             z(5,i)=-.001;
% %         end
% %         if z(5,i)>.001
% %             z(5,i)=.001;
% %         end
%         sigma_alpha_y(i) = 2e-1;
%         
% %         if z(1,i)<-.001
% %             z(1,i)=-.001;
% %         end
% %         if z(1,i)>.001
% %             z(1,i)=.001;
% %         end
%         
%         sigma_a_x(i) = 7e-1;
%         
%         sigma_a_z(i) = 7e-1;
%         
%         % standar deviation for angular acceleration noise
%         sigma_alpha_x(i) = 2e-1;
%         sigma_alpha_z(i) = 2e-1;
%         % standar deviation for linear acceleration noise
%         sigma_a_y(i) = 7e-1;
%         
%         sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
%         sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
%         
%     end
%     
% end


if strcmp(sequence,'11')
    
    for i=1:size(z,2)
        
        if (2<=i && i<446) || (537<=i && i<654) 
        
            sigma_alpha_y(i) = 2e-3;
        else
            %constant
            if z(5,i)<-.001
                z(5,i)=-.001;
            end
            if z(5,i)>.001
                z(5,i)=.001;
            end
            sigma_alpha_y(i) = 2e-6;
        end
        
        
if (324<=i && i<446) || (537<=i && i<654)  
            
            sigma_a_x(i) = 2e-4;
        else
            %constant
            sigma_a_x(i) = 7e-5;
        end
        
        
        sigma_a_z(i) = 2e-4;
        
        % standar deviation for angular acceleration noise
        sigma_alpha_x(i) = 2e-6;
        sigma_alpha_z(i) = 2e-6;
        % standar deviation for linear acceleration noise
        sigma_a_y(i) = 7e-5;
        
        sigma_alpha(:,i)=[sigma_alpha_x(i) sigma_alpha_y(i) sigma_alpha_z(i)]';
        sigma_a(:,i)=[sigma_a_x(i) sigma_a_y(i) sigma_a_z(i)]';
    end
    
end


end

