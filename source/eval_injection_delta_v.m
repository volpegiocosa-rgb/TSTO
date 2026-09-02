function required_delta_v = eval_injection_delta_v( ...
    r0, v0, apogee_target, perigee_target, inclination_target, ENV)
%EVAL_INJECTION_DELTA_V Required impulsive delta-v toward a target orbit.
%
% Pure geometry: given the current inertial state (r0, v0) and a target
% Keplerian orbit (apogee/perigee altitude, inclination), returns the
% minimum-magnitude impulsive delta-v vector, at r0, compatible with the
% target orbit. Extracted from injection_target_orbit.m so that the
% required-delta-v DIRECTION can be reused (e.g. by guidance.m case 8 for
% reporting) without duplicating the orbital-mechanics geometry and
% without needing mass/propellant/engine data (which only affect how much
% of this delta-v can actually be delivered, not its direction).
%
% INPUTS
%   r0                    [3x1] [m]     inertial position at the burn
%   v0                    [3x1] [m/s]   inertial velocity before the burn
%   apogee_target         [m]           target altitude above ENV.Req
%   perigee_target        [m]           target altitude above ENV.Req
%   inclination_target    [rad]         target inclination in [0, pi]
%   ENV.mu                [m^3/s^2]     gravitational parameter
%   ENV.Req                [m]           reference equatorial radius
%
% OUTPUT
%   required_delta_v      [3x1] [m/s]   v_target_candidate - v0 (minimum |.|)

    r0 = r0(:);
    v0 = v0(:);

    vf_candidates = target_velocity_candidates( ...
        r0, v0, apogee_target, perigee_target, ...
        inclination_target, ENV);

    required_delta_v = ...
        select_minimum_dv_candidate(vf_candidates, v0);

end


function vf_candidates = target_velocity_candidates( ...
    r0, v0, apogee_target, perigee_target, ...
    inclination_target, ENV)
%TARGET_VELOCITY_CANDIDATES Generate up to four compatible velocities.

    geometry_tolerance = 1.0e-12;
    eccentricity_tolerance = 1.0e-12;

    rp = ENV.Req + perigee_target;
    ra = ENV.Req + apogee_target;

    semi_major_axis = 0.5 * (rp + ra);
    eccentricity = (ra - rp) / (ra + rp);
    semi_latus_rectum = semi_major_axis * (1.0 - eccentricity^2);

    radius = norm(r0);
    radial_direction = r0 / radius;

    h_directions = compatible_orbital_planes( ...
        radial_direction, v0, inclination_target);

    if eccentricity <= eccentricity_tolerance
        cos_true_anomaly = 1.0;
        sin_true_anomaly_candidates = 0.0;
    else
        cos_true_anomaly = ...
            (semi_latus_rectum / radius - 1.0) / eccentricity;

        anomaly_tolerance = 1.0e-10;

        if abs(cos_true_anomaly) > 1.0 + anomaly_tolerance
            error('eval_injection_delta_v:NoTrueAnomalySolution', ...
                ['The current radius does not admit a real true anomaly ', ...
                 'on the target orbit.']);
        end

        cos_true_anomaly = max(-1.0, min(1.0, cos_true_anomaly));
        abs_sin_true_anomaly = ...
            sqrt(max(0.0, 1.0 - cos_true_anomaly^2));

        if abs_sin_true_anomaly <= geometry_tolerance
            sin_true_anomaly_candidates = 0.0;
        else
            sin_true_anomaly_candidates = [ ...
                abs_sin_true_anomaly, -abs_sin_true_anomaly];
        end
    end

    velocity_scale = sqrt(ENV.mu / semi_latus_rectum);
    transverse_velocity = velocity_scale * ...
        (1.0 + eccentricity * cos_true_anomaly);

    number_of_planes = size(h_directions, 2);
    number_of_branches = numel(sin_true_anomaly_candidates);
    number_of_candidates = number_of_planes * number_of_branches;

    vf_candidates = zeros(3, number_of_candidates);
    candidate_index = 0;

    for plane_index = 1:number_of_planes
        h_direction = h_directions(:, plane_index);
        transverse_direction = cross(h_direction, radial_direction);
        transverse_direction = ...
            transverse_direction / norm(transverse_direction);

        for branch_index = 1:number_of_branches
            sin_true_anomaly = ...
                sin_true_anomaly_candidates(branch_index);

            radial_velocity = velocity_scale * ...
                eccentricity * sin_true_anomaly;

            candidate_index = candidate_index + 1;
            vf_candidates(:, candidate_index) = ...
                radial_velocity * radial_direction + ...
                transverse_velocity * transverse_direction;
        end
    end

    vf_candidates = remove_duplicate_columns(vf_candidates, 1.0e-9);

end


function h_candidates = compatible_orbital_planes( ...
    r_hat, v0, inclination_target)
%COMPATIBLE_ORBITAL_PLANES Return compatible angular-momentum directions.
%
% Each returned column satisfies:
%   dot(h_hat, r_hat) = 0
%   h_hat(3)          = cos(inclination_target)
%   norm(h_hat)       = 1

    tolerance = 1.0e-12;
    feasibility_tolerance = 1.0e-10;
    z_hat = [0.0; 0.0; 1.0];

    projected_z = z_hat - dot(z_hat, r_hat) * r_hat;
    projected_z_norm = norm(projected_z);
    cos_inclination = cos(inclination_target);

    if projected_z_norm <= tolerance
        if abs(cos_inclination) > tolerance
            error('eval_injection_delta_v:IncompatibleInclination', ...
                ['A position on the polar axis is compatible only with ', ...
                 'a polar orbit.']);
        end

        h_candidates = select_polar_plane(r_hat, v0, tolerance);
        return;
    end

    projected_z_hat = projected_z / projected_z_norm;
    alpha = cos_inclination / projected_z_norm;

    if abs(alpha) > 1.0 + feasibility_tolerance
        geocentric_latitude = asin(max(-1.0, min(1.0, r_hat(3))));

        error('eval_injection_delta_v:IncompatibleInclination', ...
            ['The current geocentric latitude %.12g rad is not ', ...
             'compatible with target inclination %.12g rad.'], ...
            geocentric_latitude, inclination_target);
    end

    alpha = max(-1.0, min(1.0, alpha));
    beta = sqrt(max(0.0, 1.0 - alpha^2));

    secondary_direction = cross(r_hat, projected_z_hat);
    secondary_direction = ...
        secondary_direction / norm(secondary_direction);

    h_1 = alpha * projected_z_hat + beta * secondary_direction;
    h_2 = alpha * projected_z_hat - beta * secondary_direction;

    h_1 = h_1 / norm(h_1);
    h_2 = h_2 / norm(h_2);

    if norm(h_1 - h_2) <= tolerance
        h_candidates = h_1;
    else
        h_candidates = [h_1, h_2];
    end

end


function h_hat = select_polar_plane(r_hat, v0, tolerance)
%SELECT_POLAR_PLANE Select the polar plane closest to the current state.

    current_h = cross(r_hat, v0);
    current_h = current_h - dot(current_h, r_hat) * r_hat;

    if norm(current_h) > tolerance
        h_hat = current_h / norm(current_h);
        return;
    end

    reference_direction = [1.0; 0.0; 0.0];
    reference_direction = reference_direction - ...
        dot(reference_direction, r_hat) * r_hat;

    if norm(reference_direction) <= tolerance
        reference_direction = [0.0; 1.0; 0.0];
        reference_direction = reference_direction - ...
            dot(reference_direction, r_hat) * r_hat;
    end

    h_hat = reference_direction / norm(reference_direction);

end


function unique_columns = remove_duplicate_columns(matrix, tolerance)
%REMOVE_DUPLICATE_COLUMNS Keep the first occurrence of each candidate.

    number_of_columns = size(matrix, 2);
    keep_column = true(1, number_of_columns);

    for current_index = 2:number_of_columns
        kept_indices = find(keep_column(1:current_index - 1));
        previous_columns = matrix(:, kept_indices);

        distances = vecnorm( ...
            previous_columns - matrix(:, current_index), 2, 1);

        if any(distances <= tolerance)
            keep_column(current_index) = false;
        end
    end

    unique_columns = matrix(:, keep_column);

end


function required_delta_v = ...
    select_minimum_dv_candidate(vf_candidates, v0)
%SELECT_MINIMUM_DV_CANDIDATE Select the candidate closest to v0.

    if isempty(vf_candidates)
        error('eval_injection_delta_v:NoVelocityCandidate', ...
            'No compatible target velocity was generated.');
    end

    delta_v_vectors = vf_candidates - v0;
    delta_v_norms = vecnorm(delta_v_vectors, 2, 1);

    [~, best_candidate_index] = min(delta_v_norms);

    required_delta_v = delta_v_vectors(:, best_candidate_index);

end
