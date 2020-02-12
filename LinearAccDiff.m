function [acc_diff_max] = LinearAccDiff(data)
% ova f-ja je maltene identicna kao AngularVelocityDiff
    acc_diff = zeros(1,length(data));
    acc_diff(1) = sqrt(data{1}.LinearAcceleration.X^2 + data{1}.LinearAcceleration.Y^2 + data{1}.LinearAcceleration.Z^2);
    for i = 2:length(data)
        acc_prev = sqrt(data{i-1}.LinearAcceleration.X^2 + data{i-1}.LinearAcceleration.Y^2 + data{i-1}.LinearAcceleration.Z^2);
        acc_diff(i) = abs(acc_prev-sqrt(data{i}.LinearAcceleration.X^2 + data{i}.LinearAcceleration.Y^2 + data{i}.LinearAcceleration.Z^2));
    end
    acc_diff_max = max(acc_diff);
end