function [alpha_diff_max,beta_diff_max,gama_diff_max] = AnglesFirstDiff(data) 
% ovom for petljom prebacujemo podatke o orjentaciji u quaternion zapisu u 
% Ojlerove uglove (u stepenima) i odmah oduzimamo od uglova sa prvog merenja, posto smo 
% rekli da su obicno ta prva merenje normalna - sustinski su se mogli i naci
% najveci uglovi samo, pod pretpostavkom da je uvek u pocetnom stanju
% orjentacija (0,0,0), ali i ovo ima smisla
    angles = zeros(3,length(data));
    for i = 1:length(data)
        angles(:,i) = (180/pi)*quat2eul([data{i}.Orientation.X data{i}.Orientation.Y data{i}.Orientation.Z data{i}.Orientation.W]); 
        angles(:,i) = abs(angles(:,i) - angles(:,1));
    end
    gama_diff_max = max(angles(1,:)); % obrnut je redosled, jer je to default-no za quat2eul
    beta_diff_max = max(angles(2,:));
    alpha_diff_max = max(angles(3,:));
end