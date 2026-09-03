function terms = pitch_model_terms(theta_rad, omega_rad_s, model)
%PITCH_MODEL_TERMS Evaluate the configurable nonlinear pitch model.

phase_i = 2.0 * theta_rad + model.inertia_phase_rad;
terms.inertia = model.J_kg_m2 .* ...
    (1.0 + model.inertia_sin_fraction .* cos(phase_i));
terms.inertia_dtheta = -2.0 * model.J_kg_m2 .* ...
    model.inertia_sin_fraction .* sin(phase_i);

phase_n = theta_rad + model.linkage_phase_rad;
terms.ratio = model.linkage_ratio .* ...
    (1.0 + model.linkage_sin_fraction .* sin(phase_n));
terms.ratio = max(terms.ratio, 0.05);

motor_offset = terms.ratio .* theta_rad;
gravity_motor = polyval(model.gravity_poly_nm, motor_offset);
gravity_motor = min(max(gravity_motor, model.gravity_min_nm), ...
    model.gravity_max_nm);
terms.gravity_generalized_nm = terms.ratio .* gravity_motor;

% These terms belong to the true-plant disturbance d. The deployment
% controller deliberately does not use them as explicit feedforward.
terms.viscous_nm = model.B_nms_rad .* omega_rad_s;
terms.coulomb_generalized_nm = terms.ratio .* model.coulomb_nm .* ...
    tanh(omega_rad_s ./ max(model.coulomb_smooth_rad_s, eps));
terms.coriolis_nm = 0.5 .* terms.inertia_dtheta .* omega_rad_s.^2;
end
