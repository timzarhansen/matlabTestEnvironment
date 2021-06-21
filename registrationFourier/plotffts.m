function [fftOutput,magnitude,phase] = plotffts(voxelData,figureNumber)
figure(figureNumber);
subplot( 1, 3, 1 )

imagesc(voxelData(:,:,size(voxelData,1)/2))
title('Voxel: '+string(figureNumber))
axis image
%% calculate 3dFFT


fftOutput = fftshift(fftn(voxelData));
subplot( 1, 3, 2 )
magnitude = abs(fftOutput);
imagesc(magnitude(:,:,size(magnitude,1)/2));
title('Magnitude Voxel: '+string(figureNumber))
axis image

immaginaryPart=imag(fftOutput);
realPart = real(fftOutput);
phase=atan2(immaginaryPart, realPart);
subplot( 1, 3, 3 )
imagesc(phase(:,:,size(phase,1)/2));
title('Phase Voxel: '+string(figureNumber))
axis image
end

