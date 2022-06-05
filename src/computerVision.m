clear ,clc, close all


load data.mat
depth(isnan(depth))=0;
depthEq= histeq(depth);

coloredDepth(:,:,1)=depthEq/3;
coloredDepth(:,:,2)=depthEq/3;
coloredDepth(:,:,3)=depthEq/3;

coloredDepth(:,:,1)=depth/3;
coloredDepth(:,:,2)=depth/3;
coloredDepth(:,:,3)=depth/3;

[L,Centers] = imsegkmeans(img,3);
B = labeloverlay(img,L);

[LS,N] = superpixels(depthEq,20);
B2 = labeloverlay(img,LS,'Transparency',0.8);
imshow(B2)
%imshow(histeq(LS/255))
%Viola Jones Algorithm
%%
[L,Centers] = imsegkmeans(B2,3);
B = labeloverlay(img,L);
subplot(2,2,1)
imshow(B)
% imhist(depthEq)
subplot(2,2,2)
imshow(img)

[L,Centers] = imsegkmeans(coloredDepth,3);
B = labeloverlay(coloredDepth,L);

subplot(2,2,3)
imshow(B)
% imhist(depthEq)
subplot(2,2,4)
imshow(depth)