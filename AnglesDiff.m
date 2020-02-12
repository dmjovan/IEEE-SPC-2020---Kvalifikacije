function [alpha_diff_max,beta_diff_max,gama_diff_max] = AnglesDiff(data) 
% ovom for petljom prebacujemo podatke o orjentaciji u quaternion zapisu u 
% Ojlerove uglove (u stepenima) i odmah oduzimamo od uglova sa prethodnog
% merenja, da bidimo koja je max razlika, jer to znaci da se dosta
% zarotirao
    angles = zeros(3,length(data));
    angles(:,1) = (180/pi)*quat2eul([data{1}.Orientation.X data{1}.Orientation.Y data{1}.Orientation.Z data{1}.Orientation.W]);
    for i = 2:length(data)
        angles_prev = (180/pi)*quat2eul([data{i-1}.Orientation.X data{i-1}.Orientation.Y data{i-1}.Orientation.Z data{i-1}.Orientation.W]);
        angles(:,i) = abs( angles_prev - (180/pi)*quat2eul([data{i}.Orientation.X data{i}.Orientation.Y data{i}.Orientation.Z data{i}.Orientation.W])); 
    end
    gama_diff_max = max(angles(1,:)); % obrnut je redosled, jer je to default-no za quat2eul
    beta_diff_max = max(angles(2,:));
    alpha_diff_max = max(angles(3,:));
end