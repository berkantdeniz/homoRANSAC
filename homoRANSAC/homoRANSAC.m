function H = homoRANSAC(p1, p2)

assert(all(size(p1) == size(p2)));  % input matrices are of equal size
assert(size(p1, 2) == 2);  % input matrices each have two columns
assert(size(p1, 1) >= 4);  % Need at least 4  points

numIter = 100;
maxDist = 3;
maxInliers = 0;
bestH = zeros(3,3);

for i = 1:numIter %loops numIter = 100 times
    inds = randperm(size(p1, 1), 4); %inds is a vector of 4 random unique integers in [1, n]
    H1 = homographyCalc(p1(inds,:),p2(inds,:)); %calculate homography with 4 inds
    inliers = 0;
    for j = 1:size(p1,1)
        A = H1 * [p2(j,:),1]'; %A is 3xn matrix and H is a 3x3 matrix
        dist = sqrt(sum((A-[p1(j,:),1]').^2)); %A is nx3 matrices and dist is a 1xn matrix
        if(dist < maxDist)
            inliers = inliers + 1;
        end
    end
    if(inliers > maxInliers)
        bestH = H1;
        maxInliers = inliers;
    end
end

inlierpts = zeros(maxInliers,1);
k = 1;

for l = 1:size(p1,1)
    C = bestH * [p2(l,:),1]';
    dist = sqrt(sum((C-[p1(l,:),1]').^2));
    if(dist < maxDist)
        inlierpts(k) = l;
        k = k + 1;
    end
end

H = homographyCalc(p1(inlierpts,:),p2(inlierpts,:));

end

function H = homographyCalc(p1, p2)

assert(all(size(p1) == size(p2)));
assert(size(p1, 2) == 2);

n = size(p1, 1);
if n < 4                                %% To estimate Homography matrix we need at least 4 points
    error('Need at least 4 matching points');
end
H = zeros(3, 3);  % Homography matrix to be returned

M = zeros(n*3,9);
Lambda = zeros(n*3,1);
for i=1:n
    M(3*(i-1)+1,1:3) = [p2(i,:),1];
    M(3*(i-1)+2,4:6) = [p2(i,:),1];
    M(3*(i-1)+3,7:9) = [p2(i,:),1];
    Lambda(3*(i-1)+1:3*(i-1)+3) = [p1(i,:),1];
end
x = (M\Lambda)';
H = [x(1:3); x(4:6); x(7:9)];

end
