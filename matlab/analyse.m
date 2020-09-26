[A, B, C, D] = load_model();

Q = [C; C*A; C*A^2; C*A^3];
rQ = rank(Q);

disp("Rank of observability matrix:");
disp(rQ);
if rQ ~= 4
    disp("Not fully observable");
    disp("Unobservable states:");
    disp(null(Q));
else
    disp("Fully observable");
end

P = [B, A*B, A^2*B, A^3*B];
rP = rank(P);
disp("Rank of controllability matrix:");
disp(rP);
if rP ~= 4
    disp("Not fully controllable");
    disp("Uncontrollable states:");
    disp(null(transpose(P)));
else
    disp("Fully controllable");
end