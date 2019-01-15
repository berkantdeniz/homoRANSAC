
%% Initialize images

clear all; close all; clc;

outputDirect = 'D:\Burak\Senior_402\EE417\Project\Project_v1';
imageDir = 'D:\Burak\Senior_402\EE417\Project\Project_v1\4';
imageSet = imageDatastore(imageDir);

%% Extract feature points with SIFT


% Compute feature points
for i = 1 : length(imageSet.Files)
    inputImage = single(readimage(imageSet,i));
    [keyPoints{i}, descriptors{i}] = vl_sift(inputImage);
end


%% Compute homographies

for i=1:length(imageSet.Files)-1
    
    % Assign relevant data structures to variables for convenience
    descriptors1 = descriptors{i};
    descriptors2 = descriptors{i + 1};
    keySet1 = keyPoints{i};
    keySet2 = keyPoints{i + 1};
    
    % Find matching feature points between current two images. 
    [matches, scores] = vl_ubcmatch(descriptors1, descriptors2) ;
    img1_FeaturePTS = keySet1([2 1], matches(1, :))';
    img2_FeaturePTS = keySet2([2 1], matches(2, :))';
    
    homoList{i} = homoRANSAC(img1_FeaturePTS, img2_FeaturePTS);
end


%% Warp images

homoMatrix(1) = {eye(3)};
for i = 2: size(homoList, 2) + 1
    homoMatrix{i} = homoMatrix{i-1} * homoList{i - 1};
    
end
homoMap = homoMatrix;


% Compute the size of the output panorama image
minRow = 1;
minCol = 1;
maxRow = 0;
maxCol = 0;

% for each input image
for i=1:length(homoMap)
    currentImage = readimage(imageSet, i);
    [rows,cols,~] = size(currentImage);
    
    pointMatrix = cat(3, [1,1,1]', [1,cols,1]', [rows, 1,1]', [rows,cols,1]');
    
    % Map each of the 4 corner's coordinates into the coordinate system of
    % the reference image
    for j=1:4
        result = homoMap{i}*pointMatrix(:,:,j);
        
        minRow = floor(min(minRow, result(1)));
        minCol = floor(min(minCol, result(2)));
        maxRow = ceil(max(maxRow, result(1)));
        maxCol = ceil(max(maxCol, result(2)));
    end
    
end

% Calculate output image size
panoramaHeight = maxRow - minRow + 1;
panoramaWidth = maxCol - minCol + 1;

% Calculate offset of the upper-left corner of the reference image relative
% to the upper-left corner of the output image
rowOffset = 1 - minRow;
colOffset = 1 - minCol;

% Perform inverse mapping for each input image
for i=1:length(homoMap)
    
    % Create a list of all pixels' coordinates in output image
    [xCord,yCord] = meshgrid(1:panoramaWidth, 1:panoramaHeight);
    % Create list of all row coordinates and column coordinates in separate
    % vectors, x and y, including offset
    xCord = reshape(xCord,1,[]) - colOffset;
    yCord = reshape(yCord,1,[]) - rowOffset;
    
    % Create homogeneous coordinates for each pixel in output image
    panoramaPTS(1,:) = yCord;
    panoramaPTS(2,:) = xCord;
    panoramaPTS(3,:) = ones(1,size(panoramaPTS,2));
    
    % Perform inverse warp to compute coordinates in current input image
    imageCord = homoMap{i}\panoramaPTS;
    rowCord = reshape(imageCord(1,:),panoramaHeight, panoramaWidth);
    colCords = reshape(imageCord(2,:),panoramaHeight, panoramaWidth);
    
    currentImage = im2double(readimage(imageSet, i));
    
    % Bilinear interpolate
    currentWarpedImage = zeros(panoramaHeight, panoramaWidth);
    currentWarpedImage(:, :) = interp2(currentImage(:,:),colCords, rowCord, 'linear', 0);

   
    warpedImages{i} = currentWarpedImage;
    
end

%% Blend images
panoramaImage = zeros(panoramaHeight, panoramaWidth, 3);


panoramaImage = warpedImages{1};

for i = 2 : length(warpedImages)
    panoramaImage = blendg(panoramaImage, warpedImages{i});
end
imshow(panoramaImage)

imwrite(panoramaImage, fullfile(outputDirect, 'EE417_Project4.jpg'));
