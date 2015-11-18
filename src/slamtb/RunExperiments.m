%% Script to run experiments via global variables (bad!)

global currentSigma
global posNorm
global eulNorm

global NumFeaturesToUse
global applyFeatureReduction

clear all
sigmaVals = [.5, 1, 1.5, 2, 2.5];
numAnchors = [3 ,4, 5, 6, 7, 8, 10, 12];



resultsPNorm = zeros(length(sigmaVals), length(numAnchors));
resultsENorm = zeros(length(sigmaVals), length(numAnchors));

cnt = 0;

for sV = 1:length(sigmaVals)
    currentSigma = sigmaVals(sV);
    
    applyFeatureReduction = false;
    
    slamtb;
    p_without = posNorm;
    e_without = eulNorm;
    
    
    for nAAA = 1:length(numAnchors)
        NumFeaturesToUse = numAnchors(nAAA);
        cnt = cnt + 1       
        
        applyFeatureReduction = true;
        slamtb;
        p_with = posNorm;
        e_with = eulNorm;
        
        str = sprintf('Sigma = %.4f, Anchorss = %.4f, p_without = %.4f, e_without = %.4f, p_with = %.4f, e_with = %.4f',...
            currentSigma, NumFeaturesToUse, p_without, e_without, p_with, e_with);
        fh = fopen('Results04.txt', 'a');
        fprintf(fh, [str '\n']);
        fclose(fh);
        
        resultsPNorm(sV, nAAA) = p_with<p_without;
        resultsENorm(sV, nAAA) = e_with<e_without;
        
    end
    
end







fclose(fh);

