function [v_diff_max] = SensorLinearVelocityDiff(data)
% funkcija vraca maksimalnu apsolutnu razliku izmedju uzastopnih linearnih
% brzina merenih GPS-om
    v_diff = zeros(1,length(data));
    v_diff(1) = sqrt(data{1}.Twist.Twist.Linear.X^2 + data{1}.Twist.Twist.Linear.Y^2 + data{1}.Twist.Twist.Linear.Z^2);
    for i = 2:length(data)
        v_prev = sqrt(data{i-1}.Twist.Twist.Linear.X^2 + data{i-1}.Twist.Twist.Linear.Y^2 + data{i-1}.Twist.Twist.Linear.Z^2);
        v_diff(i) = abs( v_prev - sqrt(data{i}.Twist.Twist.Linear.X^2 + data{i}.Twist.Twist.Linear.Y^2 + data{i}.Twist.Twist.Linear.Z^2));
    end
    v_diff_max = max(v_diff);
end