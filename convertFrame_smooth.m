% Manage coordinate system and filtering

% change coordinate frame
q0 = [cosd(-90/2) sind(-90/2) 0 0];
for fname = {'qshin', 'qfoot'}
    fprintf('Rotating quaternion %s\n', fname{:})
    var.(fname{:}) = quatmultiply(quatmultiply(quatinv(q0), var.(fname{:})), q0);
end

for fname = setdiff(fields(var), {'qshin', 'qfoot'})'
    fprintf('Rotating %s\n', fname{:})
    var.(fname{:}) = quatrotate(q0, var.(fname{:}));
end

% convert to timeseries
rows = 10 : size(var.qshin, 1) - 10;
time = (0 : length(find(rows))-1)' / m.Fs;
for fname = fields(var)'
    tvar.(fname{:}) = timeseries(var.(fname{:})(rows,:), time, 'Name', fname{:});
end

% figure
% plot(tvar.wshin)

%% Smooth trajectories

n = size(tvar.qshin.Data, 1);

phase = 2*pi*0.6 * tvar.qshin.Time;
orders = 1 : 17;

basis = [cos(phase * orders) ./ orders.^2, sin(phase * orders) ./ orders.^2, phase ones(n,1)];

% % experiment several basis
% y = tvar.wshin.Data;
% x = basis \ y;
% figure, plot([basis*x, y])

for fname = fields(tvar)'
    name = fname{:};
    x = basis \ tvar.(name).Data;
    tvar.(name).Data = basis*x;
end