function [ imgBlended ] = blendg( img1, img2 )

assert(all(size(img1) == size(img2)));

imgBlended = zeros(size(img1), 'like', img1);

alpha1 = grey2alpha(img1);
alpha2 = grey2alpha(img2);

outChannel = (alpha1.*img1(:,:) + alpha2.*img2(:,:)) ...
    ./(alpha1 + alpha2);

imgBlended(:,:) = outChannel;


end

function imgAlpha = grey2alpha(img, epsilon)


switch nargin
    case 1
        epsilon = 0.001;
end

binaryImg = im2bw(img, epsilon);

complementImg = imcomplement(binaryImg);
imgDistance = bwdist(complementImg,'euclidean');
imgAlpha = rescale(imgDistance, epsilon, 1);


end