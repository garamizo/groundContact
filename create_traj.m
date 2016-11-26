load('data.mat')

%%
kk = 3;

b = fir1(30, 30/125);

w = data(kk).anklew(:,:);
w(isnan(w)) = 0;

pp = 100;
plot([w(:,1:50:end) filtfilt(b, 1, w(:,1:50:end))])

