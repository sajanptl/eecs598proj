function [Rstar] = submodularLearning(X, R, Kstar)
%% submodularLearning
%  
%  Upgrades Kstar of the remaning features to anchors after the incremental
%  SVD calculation phase of GF-SLAM. This is called when the thresholding
%  of the singular values results in a less than desired number of good
%  features (anchors).
%   hm
%  INPUTS:  X   n by m SOM matrix for the features with high observability 
%               scores (passing the threshold score), where n >= m. These 
%               are for the features that are going to become anchors after
%               the incremental SVD calulations, but there are less than
%               (Ka - 2) features, where Ka is the desired number of good
%               features for GF-SLAM.
%
%           R   a map container of K elements where the key is the index of
%               the row block/vector Rk and the element is the actual 
%               1 by m candidate row vector Rk. The index should be treated
%               as the index of the row as if R was a K by m matrix. An
%               alternative way of viewing the index is that if each Rk is
%               unique to a feature, then the map key can be the feature
%               id.
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

while (length(Rstar) < Kstar)
   %% calculate r = argmax FsigMin(X, R)
   [r, idx] = optFsigMin(X, R);
   
   %% add r to Rstar set
   Rstar(idx) = r;
   
   %% remove r from R
   R.remove(idx);
end
end

%%
%
function [r, idx] = optFsigMin(X, R)
%% optFsigMin
% peroforms argmin FminSig([X', R(k)']')
%%
[n, m] = size(X);
assert(n >= m);

x = [X; zeros(1,m)];
keySet = keys(R);
minVals = continer.Map(keySet, zeros(size(keySet),1));

for k = keySet
    %% set last row of expanded x SOM to R(k)
    x(n+1,:) = R(k);
    
    %% compute svd of x to get vector of singular values in decreasing order
    s = svd(x, 'econ');
    
    %% add min singular value to minVals
    minVals(k) = s(end);
end

%% find max of min signular values and the corresponding k and R(k)
maxSig = 0;
for k = keySet
   if (maxSig < minVals(k))
       idx = k;
       r = R(k);
       maxSig = minVals;
   end
end
end
