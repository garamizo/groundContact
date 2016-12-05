% plate dimensions
phei = 28.8e-3;

% plate base impedance
kplate = 100;
bplate = 0.1;

% foot dimensions
Cdist = 15e-2;
wid = 5e-2;
hei = 6e-2;
th = (pi/2:0.5:3*pi/2)';
r = [cos(th) sin(th) -ones(size(th))*hei/wid
     -cos(-th)+Cdist/wid sin(-th) -ones(size(th))*hei/wid]' * wid;
 
% shin dimensions
shinL = 40e-2;
shinr = 5e-2;

% ankle impedance
k = 200;
b = 5;

% shin trajectory
load('stance_trajectory4.mat')
tend = tvar.qshin.Time(end);

% tvar.tshin.Data(:,3) = tvar.tshin.Data(:,3) - 10e-2*(exp(-3*(tvar.tshin.Time - 0.6).^2) - 0.5);
tvar.tshin.Data = tvar.tshin.Data - [-0.3227, -0.1955, 0.07];

% tvar.vshin.Data = tvar.vshin.Data*0;
% 
% tvar.qshin.Data = quatmultiply(quatinv(tvar.qshin.Data), tvar.qshin.Data);
% tvar.wshin.Data = tvar.wshin.Data*0;

% %%
% load('stance_trajectory.mat')
% 
% for fname = fields(tvar)'
%     name = fname{1};
%     tvar.(name).Time = tvar.(name).Time - tvar.(name).Time(1);
% end