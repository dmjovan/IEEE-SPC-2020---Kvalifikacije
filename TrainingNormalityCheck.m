function [training_valid] = TrainingNormalityCheck(training_data,data)

    training_valid = zeros(1,length(data));
    for i = 1:length(data)
        training_valid(i) = abs(data(i)-training_data(1))< abs(data(i)-training_data(2));
    end                   % dakle, ako je apsolutna razlika merenja sa 
                          % normalnog tregning manja od aps. razlike 
                          % merenja sa abnormalnim treningom, onda je snimak
                          % normalan
end