function [fftOutput,magnitude,phase] = plotffts2D(voxelData,figureNumber)
figure(figureNumber);
subplot( 1, 3, 1 )

imagesc(voxelData)
title('Voxel: '+string(figureNumber))
axis image

%% calculate 3dFFT


fftOutput = fftshift(fftn(voxelData));
%fftOutput = (fftn(voxelData));

subplot( 1, 3, 2 )
magnitude = abs(fftOutput);
imagesc(magnitude);
%imshow(magnitude(:,:,size(magnitude,1)/2));
title('Magnitude Voxel: '+string(figureNumber))
axis image

immaginaryPart=imag(fftOutput);
realPart = real(fftOutput);
phase=atan2(immaginaryPart, realPart);
subplot( 1, 3, 3 )
imagesc(phase);
title('Phase Voxel: '+string(figureNumber))
axis image
end

