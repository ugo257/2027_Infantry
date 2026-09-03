function linear = pitch_linearize_model(cfg, theta0_rad)
%PITCH_LINEARIZE_MODEL Linearize the nominal feedforward-cancelled plant.

terms = pitch_model_terms(theta0_rad, 0.0, cfg.model);
A = [0.0, 1.0; 0.0, 0.0];
B = [0.0; terms.ratio * cfg.model.torque_gain / terms.inertia];
C = eye(2);
D = zeros(2, 1);

plant_c = ss(A, B, C, D);
plant_d = c2d(plant_c, cfg.sim.Ts, 'zoh');

linear.theta0_rad = theta0_rad;
linear.terms = terms;
linear.A = A;
linear.B = B;
linear.C = C;
linear.D = D;
linear.plant_c = plant_c;
linear.plant_d = plant_d;
linear.controllability_rank = rank(ctrb(A, B));
linear.observability_rank = rank(obsv(A, C));
end
