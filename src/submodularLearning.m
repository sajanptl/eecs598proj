function [Rstar] = submodularLearning(X, R, Kstar)
%% submodularLearning
%  
%  Upgrades Kstar of the remaning features to anchors after the incremental
%  SVD calculation phase of GF-SLAM. This is called when the thresholding
%  of the singular values results in a less than desired number of good
%  features (anchors).
%   
%  INPUTS:  X   n by m SOM matrix for the features with high observability 
%               scores (passing the threshold score), where n >= m. These 
%               are for the features that are going to become anchors after
%               the incremental SVD calulations, but there are less than
%               (Ka - 2) features, where Ka is the desired number of good
%               features for GF-SLAM.
%
%           R   a map container of K elements where the key is the index of
%               the row block/vector Rk and the element is the actual 
%               1 by m candidate row vector Rk
%
%           Kstar   The number of row blocks/vectors to add to X.
%
% OUTPUTS:  Rstar   The map of Kstar optimal row vectors to add to the GF 
%                   SOM X to choose Kstar fectures to upgrade to anchors.
%
% AUTHOR: Sajan Patel (sajanptl@umich.edu), 15 November 2015.
%         Part of the EECS 598-001 F15 Final Project with Shurjo Banerjee
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
[n, m] = size(X);
assert(n >= m);

Rstar = containers.Map({},[]);
numAdded = 0; % number of Rstar rows added, right now...this 
while (numAdded < Kstar)
%while (length(Rstar) < Kstar)
   %% calculate r = argmax FsigMin(X, R)
   [r, idx] = optFsigMin(X, R);
   
   %% add r to Rstar set (IF IDX IS THE FEATURE ID, USE IDX AS THE KEY)
   Rstar(numAdded + 1) = r;
   numAdded = numAdded + 1;
   %Rstar(idx) = r;
   
   %% remove r from R
   R.remove(idx);
end

end

function [r, idx] = optFsigMin(X, R)
%% do the argmax of the min F here
maxSig = 0;
[n, m] = size(X);
assert(n >= m);

x = [X; zeros(1,m)];

keySet = keys(R);
minVals = continer.Map(keySet, zeros(size(keySet),1));

for k = keySet
    %% set last row of expanded x SOM to R(k)
    x(n+1,:) = R(k);
    
    %% compute svd of x
    
    %% find minSig and add to minVals
    
end

%% find max of minSig and the corresponding k and R(k)
for k = keySet
   if (maxSig < minVals(k))
       idx = k;
       r = R(k);
       maxSig = minVals;
   end
end
end
