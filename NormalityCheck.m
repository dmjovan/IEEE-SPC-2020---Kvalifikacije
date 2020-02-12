function [valid] = NormalityCheck(data)
    data_mean = mean(data);
    valid = ones(1,length(data));
    for i = 1:length(data)
        if  data(i)>=data_mean % dakle,vrednosti preko srednje vrednosti su neregularne
            valid(i) = 0;
        end
    end
end