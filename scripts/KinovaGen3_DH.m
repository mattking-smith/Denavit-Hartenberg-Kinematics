% DH table for Kinova Gen3
% Table found on page 205 of Kinova-Gen-3_User-Guide.pdf
DH.alpha(1) = pi;
DH.alpha(2) = pi/2;
DH.alpha(3) = pi/2;
DH.alpha(4) = pi/2;
DH.alpha(5) = pi/2;
DH.alpha(6) = pi/2;
DH.alpha(7) = pi/2;
DH.alpha(8) = pi;

DH.a(1:8) = 0;

DH.d(1) = 0;
DH.d(2) = - (0.1564 + 0.1284);
DH.d(3) = - (0.0054 + 0.0064);
DH.d(4) = - (0.2104 + 0.2104);
DH.d(5) = - (0.0064 + 0.0064);
DH.d(6) = - (0.2084 + 0.1059);
DH.d(7) = 0;
DH.d(8) = - (0.1059 + 0.0615);

DH.manipulator = 'KinovaGen3';
% Must add pi to some of the joint angles for this convention
DH.theta(1) = 0;
DH.theta(2) = 0;
DH.theta(3:8) = pi;
DH.param = 'DH';

% Modified DH table
modDH.manipulator = 'KinovaGen3';
modDH.param = 'modDH';
modDH.a(1:7) = 0;

modDH.alpha(1) = pi;
modDH.alpha(2) = pi/2;
modDH.alpha(3) = -pi/2;
modDH.alpha(4) = pi/2;
modDH.alpha(5) = -pi/2;
modDH.alpha(6) = pi/2;
modDH.alpha(7) = -pi/2;

modDH.d(1) = - (0.1564 + 0.1284);
modDH.d(2) = - (0.0054 + 0.0064);
modDH.d(3) = - (0.2104 + 0.2104);
modDH.d(4) = - (0.0064 + 0.0064);
modDH.d(5) = - (0.2084 + 0.1059);
modDH.d(6) = 0;
modDH.d(7) = - (0.1059 + 0.0615);