function TAUS = ComputeObsScoreFromQSOM(Q_SOM, currentFeatures, RowsPerFeature, time_start, time_end)

    timeDiff = time_end - time_start;

    TAUS = zeros(1,length(currentFeatures));
    for i = 1:length(currentFeatures)
%         featureMatrix = Q_SOM((currentFeatures(i)-1)*RowsPerFeature + 1: (currentFeatures(i))*RowsPerFeature , (time_start-1)*13+1:time_end*13);
        
        featureMatrix = Q_SOM((currentFeatures(i)-1)*RowsPerFeature + 1: (currentFeatures(i))*RowsPerFeature , ...
            end - (timeDiff +1)*13 + 1:end);

        [~,S,~] = svds(featureMatrix);

        singVals = diag(S);
        singVals(singVals<1e-5) = [];
        tau = min(singVals);
        TAUS(i) = tau;
    end

end