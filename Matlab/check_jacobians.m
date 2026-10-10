function check_jacobians(include_symbolic)
% Compare DH/PoE, finite differences, and optionally exact symbolic results.
if nargin == 0
    include_symbolic = true;
end
g = robot_dimensions();
custom = g;
names = fieldnames(g);
for i = 1:numel(names)
    custom.(names{i}) = g.(names{i}) * (0.7 + 0.1*i);
end
geometries = {g,custom};
stream = RandStream('mt19937ar','Seed',42);
Q = [zeros(6,1), (pi/2)*eye(6), -(pi/2)*eye(6), ...
    [20;-35;50;15;-40;60]*pi/180, -pi+2*pi*rand(stream,6,100)];
max_error = 0;
max_fd = 0;

for geometry = 1:numel(geometries)
    dh = dh_data(geometries{geometry});
    poe = poe_data(geometries{geometry});
    for sample = 1:size(Q,2)
        q = Q(:,sample);
        for point = [5,6]
            for frame = 0:6
                Jdh = dh_jacobian(q,frame,point,dh);
                Jpoe = poe_jacobian(q,frame,point,poe);
                max_error = max(max_error,norm(Jdh-Jpoe,'fro'));
            end
        end
    end
    q = Q(:,14);
    for point = [5,6]
        Jdh = dh_jacobian(q,0,point,dh);
        Jpoe = poe_jacobian(q,0,point,poe);
        assert(isequal(Jdh,dh_jacobian_numerical(q.',0,point,dh)));
        assert(isequal(Jpoe,poe_jacobian_numerical(q.',0,point,poe)));
        Jfd = finite_difference(q,point,dh);
        [~,~,~,T0] = dh_fk(q,dh);
        for frame = 0:6
            R = T0(1:3,1:3,frame+1);
            J = dh_jacobian(q,frame,point,dh);
            max_fd = max(max_fd,norm(J-blkdiag(R.',R.')*Jfd,'fro'));
        end
    end
    J = dh_jacobian(q,3,5,dh);
    assert(norm(J(1:3,4:6),'fro') < 1e-10);
    assert(abs(norm(J(4:6,6))-1) < 1e-10);

    % Check the closed-form PoE joint motions against matrix exponentials.
    F = eye(4);
    for i = 1:6
        F = F * expm(poe.xi_hat(:,:,i)*q(i));
    end
    assert(norm(poe_fk(q,poe)-F*poe.M,'fro') < 1e-10);
end
assert(max_error < 1e-10,'Numerical DH/PoE Jacobians disagree.');
assert(max_fd < 1e-7,'Jacobian disagrees with differentiated forward kinematics.');
fprintf('Jacobian DH-PoE: %.3e; finite differences: %.3e.\n',max_error,max_fd);
fprintf('%d configurations, two geometries, points 5/6, frames 0:6.\n',size(Q,2));

if include_symbolic
    check_symbolic(geometries,Q(:,[1,14,15]));
end
end

function J = finite_difference(q,point,dh)
h = 1e-6;
T = dh_fk(q,dh);
J = zeros(6,6);
for i = 1:6
    delta = zeros(6,1);
    delta(i) = h;
    [Tp,~,~,Pp] = dh_fk(q+delta,dh);
    [Tm,~,~,Pm] = dh_fk(q-delta,dh);
    J(1:3,i) = (Pp(1:3,4,point+1)-Pm(1:3,4,point+1))/(2*h);
    W = ((Tp(1:3,1:3)-Tm(1:3,1:3))/(2*h))*T(1:3,1:3).';
    J(4:6,i) = [W(3,2)-W(2,3); W(1,3)-W(3,1); W(2,1)-W(1,2)]/2;
end
end

function check_symbolic(geometries,Q)
gs = robot_dimensions(true);
qs = sym('q',[6,1],'real');
fields = struct2cell(gs);
parameters = vertcat(fields{:});
dh = dh_data(gs);
poe = poe_data(gs);
cases = [0,6; 0,5; 3,5];
max_error = 0;
for c = 1:size(cases,1)
    frame = cases(c,1);
    point = cases(c,2);
    Jdh = simplify(dh_jacobian(qs,frame,point,dh));
    Jpoe = simplify(poe_jacobian(qs,frame,point,poe));
    difference = simplify(expand(Jdh-Jpoe));
    assert(isequal(difference,sym(zeros(6))), ...
        'Symbolic DH/PoE mismatch for frame %d, point %d.',frame,point);
    fprintf('Symbolic DH-PoE: frame %d, point %d agree exactly.\n',frame,point);
    evaluate_dh = matlabFunction(Jdh,'Vars',{qs,parameters});
    evaluate_poe = matlabFunction(Jpoe,'Vars',{qs,parameters});
    for geometry = 1:numel(geometries)
        values = cell2mat(struct2cell(geometries{geometry}));
        dh_num = dh_data(geometries{geometry});
        poe_num = poe_data(geometries{geometry});
        for sample = 1:size(Q,2)
            q = Q(:,sample);
            max_error = max([max_error, ...
                norm(evaluate_dh(q,values)-dh_jacobian(q,frame,point,dh_num),'fro'), ...
                norm(evaluate_poe(q,values)-poe_jacobian(q,frame,point,poe_num),'fro')]);
        end
    end
end
assert(max_error < 1e-10,'Symbolic evaluation disagrees with numerical models.');
fprintf('Symbolic vs numerical Jacobians: %.3e.\n',max_error);
end
