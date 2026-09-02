function [v_final, dv_delivered, residual_mass, ...
          apogee_reached, perigee_reached, inclination_reached] = ...
          injection_target_orbit( ...
              r0, v0, m0, available_propellant, ...
              apogee_target, perigee_target, inclination_target, ...
              ENV, ue)
%INJECTION_TARGET_ORBIT Impulsive injection toward a target Keplerian orbit.
%
% The target orbit is defined by:
%   - apogee altitude;
%   - perigee altitude;
%   - inclination.
%
% The burn position r0 may be any point of the target orbit. In the
% general case, up to four target velocities are compatible with the
% imposed apsides and inclination:
%   - two compatible orbital planes;
%   - two radial branches, with increasing or decreasing radius.
%
% The candidate requiring the minimum impulsive delta-v from v0 is used.
% If the available propellant is insufficient, all available delta-v is
% delivered in the direction of the selected maneuver.
%
% INPUTS
%   r0                    [3x1] [m]       inertial position at the burn
%   v0                    [3x1] [m/s]     inertial velocity before the burn
%   m0                    [kg]            mass before the burn
%   available_propellant  [kg]            usable propellant mass
%   apogee_target         [m]             target altitude above ENV.Req
%   perigee_target        [m]             target altitude above ENV.Req
%   inclination_target    [rad]           target inclination in [0, pi]
%   ENV.mu                [m^3/s^2]       gravitational parameter
%   ENV.Req               [m]             reference equatorial radius
%   ue                    [m/s]           effective exhaust velocity
%
% OUTPUTS
%   v_final               [3x1] [m/s]     inertial velocity after the burn
%   dv_delivered          [m/s]           delivered delta-v magnitude
%   residual_mass         [kg]            mass after the burn
%   apogee_reached        [m]             achieved altitude above ENV.Req
%   perigee_reached       [m]             achieved altitude above ENV.Req
%   inclination_reached   [rad]           achieved inclination
%
% NOTES
%   - The maneuver is impulsive.
%   - The dynamics are Keplerian and use a two-body model.
%   - For a parabolic or hyperbolic final trajectory, apogee_reached = Inf.

    validate_inputs( ...
        r0, v0, m0, available_propellant, ...
        apogee_target, perigee_target, inclination_target, ...
        ENV, ue);

    r0 = r0(:);
    v0 = v0(:);

    required_delta_v = eval_injection_delta_v( ...
        r0, v0, apogee_target, perigee_target, inclination_target, ENV);

    [v_final, dv_delivered, residual_mass] = ...
        deliver_delta_v( ...
            v0, required_delta_v, m0, available_propellant, ue);

    [apogee_reached, perigee_reached, inclination_reached] = ...
        orbital_apsides_inclination( ...
            r0, v_final, ENV.mu, ENV.Req);

end


function validate_inputs( ...
    r0, v0, m0, available_propellant, ...
    apogee_target, perigee_target, inclination_target, ...
    ENV, ue)
%VALIDATE_INPUTS Validate dimensions, values and target compatibility.

    validateattributes(r0, {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 3}, ...
        mfilename, 'r0');

    validateattributes(v0, {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 3}, ...
        mfilename, 'v0');

    validateattributes(m0, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'positive'}, ...
        mfilename, 'm0');

    validateattributes(available_propellant, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative'}, ...
        mfilename, 'available_propellant');

    validateattributes(apogee_target, {'numeric'}, ...
        {'real', 'finite', 'scalar'}, ...
        mfilename, 'apogee_target');

    validateattributes(perigee_target, {'numeric'}, ...
        {'real', 'finite', 'scalar'}, ...
        mfilename, 'perigee_target');

    validateattributes(inclination_target, {'numeric'}, ...
        {'real', 'finite', 'scalar', '>=', 0.0, '<=', pi}, ...
        mfilename, 'inclination_target');

    validateattributes(ue, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'positive'}, ...
        mfilename, 'ue');

    if ~isstruct(ENV) || ~isscalar(ENV)
        error('injection_target_orbit:InvalidEnvironment', ...
            'ENV must be a scalar structure.');
    end

    if ~isfield(ENV, 'mu') || ~isfield(ENV, 'Req')
        error('injection_target_orbit:InvalidEnvironment', ...
            'ENV must contain the fields mu and Req.');
    end

    validateattributes(ENV.mu, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'positive'}, ...
        mfilename, 'ENV.mu');

    validateattributes(ENV.Req, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'positive'}, ...
        mfilename, 'ENV.Req');

    if available_propellant >= m0
        error('injection_target_orbit:InvalidPropellantMass', ...
            'available_propellant must be strictly lower than m0.');
    end

    if perigee_target > apogee_target
        error('injection_target_orbit:InvalidTargetApsides', ...
            'perigee_target cannot exceed apogee_target.');
    end

    rp_target = ENV.Req + perigee_target;
    ra_target = ENV.Req + apogee_target;

    if rp_target <= 0.0
        error('injection_target_orbit:InvalidPerigeeRadius', ...
            'The target perigee radius must be positive.');
    end

    r_norm = norm(r0);

    if r_norm <= eps(ENV.Req)
        error('injection_target_orbit:InvalidPosition', ...
            'The norm of r0 must be greater than zero.');
    end

    radius_tolerance = max( ...
        1.0e-3, ...
        1.0e-10 * max([r_norm, rp_target, ra_target]));

    if r_norm < rp_target - radius_tolerance || ...
       r_norm > ra_target + radius_tolerance
        error('injection_target_orbit:PositionNotOnTargetOrbit', ...
            ['The current position is not compatible with the target ', ...
             'orbit. Current radius: %.6f m; target interval: ', ...
             '[%.6f, %.6f] m.'], ...
            r_norm, rp_target, ra_target);
    end

end


function [v_final, dv_delivered, residual_mass] = ...
    deliver_delta_v( ...
        v0, required_delta_v, m0, available_propellant, ue)
%DELIVER_DELTA_V Deliver the requested or maximum available delta-v.

    delta_v_tolerance = 1.0e-10;

    dry_mass = m0 - available_propellant;
    dv_available = ue * log(m0 / dry_mass);
    dv_required = norm(required_delta_v);

    if dv_required <= dv_available + delta_v_tolerance
        dv_delivered = dv_required;
        v_final = v0 + required_delta_v;
        residual_mass = m0 * exp(-dv_delivered / ue);
        residual_mass = max(dry_mass, min(m0, residual_mass));
        return;
    end

    dv_delivered = dv_available;
    residual_mass = dry_mass;

    if dv_required <= delta_v_tolerance
        v_final = v0;
        return;
    end

    delta_v_direction = required_delta_v / dv_required;
    v_final = v0 + dv_delivered * delta_v_direction;

end


function [apogee_altitude, perigee_altitude, inclination] = ...
    orbital_apsides_inclination( ...
        position, velocity, mu, reference_radius)
%ORBITAL_APSIDES_INCLINATION Compute apsides and inclination from state.

    radius = norm(position);

    if radius <= eps(reference_radius)
        error('injection_target_orbit:InvalidFinalPosition', ...
            'The final position norm must be positive.');
    end

    angular_momentum = cross(position, velocity);
    angular_momentum_norm = norm(angular_momentum);

    angular_momentum_tolerance = 1.0e-12 * radius * ...
        max(norm(velocity), sqrt(mu / radius));

    if angular_momentum_norm <= angular_momentum_tolerance
        error('injection_target_orbit:DegenerateFinalOrbit', ...
            ['The final state has nearly zero angular momentum; ', ...
             'apsides and inclination are undefined.']);
    end

    cos_inclination = angular_momentum(3) / angular_momentum_norm;
    cos_inclination = max(-1.0, min(1.0, cos_inclination));
    inclination = acos(cos_inclination);

    eccentricity_vector = ...
        cross(velocity, angular_momentum) / mu - position / radius;
    eccentricity = norm(eccentricity_vector);

    semi_latus_rectum = angular_momentum_norm^2 / mu;
    perigee_radius = semi_latus_rectum / (1.0 + eccentricity);
    perigee_altitude = perigee_radius - reference_radius;

    specific_orbital_energy = ...
        0.5 * dot(velocity, velocity) - mu / radius;

    eccentricity_tolerance = 1.0e-10;
    energy_tolerance = 1.0e-12 * mu / radius;

    is_elliptic = ...
        eccentricity < 1.0 - eccentricity_tolerance && ...
        specific_orbital_energy < -energy_tolerance;

    if is_elliptic
        apogee_radius = semi_latus_rectum / (1.0 - eccentricity);
        apogee_altitude = apogee_radius - reference_radius;
    else
        apogee_altitude = Inf;
    end

end
