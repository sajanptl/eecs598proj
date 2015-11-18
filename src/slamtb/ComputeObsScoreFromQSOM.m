function TAUS = ComputeObsScoreFromQSOM(Q_SOM, currentFeatures, RowsPerFeature, time_start, time_end, n)

    timeDiff = time_end - time_start;

    TAUS = zeros(1,length(currentFeatures));
    for i = 1:length(currentFeatures)
%         featureMatrix = Q_SOM((currentFeatures(i)-1)*RowsPerFeature + 1: (currentFeatures(i))*RowsPerFeature , :);
        
        featureMatrix = Q_SOM(currentFeatures(i):n:end,:);

        singVals = svds(featureMatrix)';

        singVals(singVals<1e-5) = [];
        tau = min(singVals);
        if ~isempty(tau)
            TAUS(i) = tau;
        else
            TAUS(i) = 0;
        end
    end

end