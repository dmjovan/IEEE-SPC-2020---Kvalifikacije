function [w_diff_max] = AngularVelocityDiff(data)
% ovom f-jom nalazim maksimalnu apsolutnu razliku izmedju dve uzastopne
% ugaone brzine - u sustini, za koliko se promeni njevog smer, doduse
% brzina u toku jedne periode odabiranja, ako je ona velika (preko definisane 
% granice) onda to predstavlja abnormalitet u snimku
    w_diff = zeros(1,length(data));
    w_diff(1) = sqrt(data{1}.AngularVelocity.X^2 + data{1}.AngularVelocity.Y^2 + data{1}.AngularVelocity.Z^2);
    for i = 2:length(data)
        w_prev = sqrt(data{i-1}.AngularVelocity.X^2 + data{i-1}.AngularVelocity.Y^2 + data{i-1}.AngularVelocity.Z^2);
        w_diff(i) = abs(w_prev - sqrt(data{i}.AngularVelocity.X^2 + data{i}.AngularVelocity.Y^2 + data{i}.AngularVelocity.Z^2));
    end
    w_diff_max = max(w_diff);
end