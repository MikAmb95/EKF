

function mdl_p = mdl_pendl()


%I = diag([5*0+12 8*0+12 2.50+12]);

m = 22; %block mass
l=1.215; %cable length

l_b = 0.8; %length_block
h_b = 0.6; %height_block
t_b = 0.2; %thickness_block

d1 = h_b/2; %height block/2

I1 = m*(t_b^2+l_b^2)/12; 
I2 = m*(h_b^2+l_b^2)/12;
I3 = m*(t_b^2+h_b^2)/12;

I = diag([I1 I2 I3]);



L(1) = Link('revolute', 'd', 0, 'a',0, 'alpha', -pi/2,'offset',-pi/2,'m',0,'I',zeros(3,3)); %first boom angle
L(2) = Link('revolute', 'd', 0, 'a',-l, 'alpha', 0,'offset',0,'m',0,'I',zeros(3,3)); %first boom angle
L(3) = Link('revolute', 'd', 0, 'a',0, 'alpha', pi/2,'offset',0,'m',0,'I',zeros(3,3)); %first boom angle
L(4) = Link('revolute', 'd', 0, 'a',0, 'alpha', -pi/2,'offset',pi/2,'m',0,'I',zeros(3,3)); %first boom angle
L(5) = Link('revolute', 'd', d1, 'a',0, 'alpha', 0,'offset',0,'m',100,'r',[0 0 0],'I',I); %first boom angle




%crane = SerialLink(L,'name','crane');
mdl_p = SerialLink(L,'base',trotx(-pi/2),'name','mdl_p');

end