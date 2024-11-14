function theta = anglesR(R,str)
% solve a 3d rotation matrix R of the form Ra(th1)*Rb(th2)*Rc(th3)
% for angles th1,th2,th3; where each of a,b c are one of x,y,z.
% input str is a three-letter string of rotation axes, such as 'yzx'.
% consecutive rotations along the same axis such as 'xxy' are not allowed.
% 1st and 3rd rotations along different axes such as 'yzx' are tait-bryan,
% along the same axis such as 'xzx' are euler.  12 possibilities in all.
% Output is the vector [th1 th2 th3] and angles are in DEGREES,
% with -180<(th1,th3)<180;  -90<th2<90 (tait-bryan), 0<th2<180 (euler).  
% see below for angle restrictions and matrices Rx,Ry,Rz
theta = zeros(1,3);
% similarity transform matrices
By = [0 0 1;0 -1 0;1 0 0];     % x<-->z  y(th)-->y(-th)
Ry90 = [0 0 1;0 1 0; -1 0 0];  % y rotation by 90 deg
C = [0 0 1;1 0 0; 0 1 0;];     % cycic, x-->y-->z-->x
signy = 1;
% tranform to RaRyRb
if str(2) == 'x'
    R = C*R/C;
elseif str(2) == 'z'
    R = C\R*C;
end
% tait-bryan, transform to RxRyRz 
if all(str=='xzy')|all(str=='yxz')|all(str=='zyx')
    R = By*R/By;
    signy = -1;
end
% euler, transform to RxRyRx 
if all(str=='xzx')|all(str=='yxy')|all(str=='zyz')
    R = Ry90*R/Ry90;
end  
if str(1)~=str(3)                 % tait-bryan
    theta(2) = signy*asind(R(1,3));
    theta(1) = atan2d(-R(2,3),R(3,3));
    theta(3) = atan2d(-R(1,2),R(1,1));  
else                              % euler
    theta(2) = acosd(R(1,1));
    theta(1) = atan2d(R(2,1),-R(3,1));
    theta(3) = atan2d(R(1,2), R(1,3));
end
end
% Rotation of an object's coordinates.  Counterclockwise
% rotation looking down at the rotation axis coming up out of the page.
%
%        [1 0  0         [c 0 s          [c -s 0        
%  Rx =   0 c -s    Ry =  0 1 0    Rz =   s  c 0
%         0 s  c]        -s 0 c]          0  0 1]
%
%  tait-bryan RxRyRz
%  [c2c3  -c2s3  s2
%    .      .   -s1c2
%    .      .    c1c2]
% 
% euler RxRyRx
%  [c2    s2s3  s2c3
%   s1s2   .     .
%  -c1s2   .     .  ]