% foot dimensions
Cdist = 20e-2;
wid = 7e-2;
hei = 12e-2;
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

tvar.tshin.Data = tvar.tshin.Data - 15e-2*(exp(-3*(tvar.tshin.Time - 0.6).^2) - 0.5);
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