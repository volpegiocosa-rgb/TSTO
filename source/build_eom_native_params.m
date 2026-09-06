function [scalars, InOl, env_alt, env_rho, env_c, env_p, aer_mach, aer_aoa, aer_cd] = build_eom_native_params(other)
% BUILD_EOM_NATIVE_PARAMS  Impacchetta da 'other' gli argomenti per
%   eom_native e phase_event_native (kernel Fortran, native/eom_core.f90 +
%   shim native/eom_oct.cc, native/phase_event_oct.cc), rif. CLAUDE.md
%   solver_project S11 Fase 5, sessione "requisito 5 minuti".
%
%   Va chiamata UNA VOLTA per fase (other.phase/isignite/GUI.active_stage
%   gia' impostati, come fa simulator.m prima dell'integrazione), non ad
%   ogni chiamata dentro rk5.m: i valori impacchettati sono costanti per
%   l'intera fase, solo (t,y) cambiano da una chiamata all'altra. Layout
%   di 'scalars' (38 elementi: 1-31 usati da eom_native, vedi header di
%   native/eom_core.f90; 32-38 aggiunti per phase_event_native, che ne
%   riusa anche alcuni tra 1-31 -- es. mu/Req/f/omega_E/last_pitch/
%   last_yaw/transition_starting/insertion_starting -- stesso vettore,
%   nessuna duplicazione):
%   32 GUI.zkick                36 MASS.Mfairing
%   33 MASS.Minert1             37 MASS.Mpayload
%   34 MASS.Minert2             38 MIS.apogee_altitude_target
%   35 MASS.MProp2
%
%   INPUT  other  : struct con ENV/AER/MOT/GUI/MIS/MASS + isignite/phase
%                   gia' impostati (stessa convenzione di eom.m/phase_event.m).
%   OUTPUT scalars, InOl, env_alt, env_rho, env_c, env_p, aer_mach,
%          aer_aoa, aer_cd : argomenti pronti per eom_native(t, y, ...) e
%          (scalars, InOl, phase) per phase_event_native(t, y, ...).

    stage = other.GUI.active_stage;

    scalars = zeros(38, 1);
    scalars(1)  = other.ENV.mu;
    scalars(2)  = other.ENV.wgs84(1);
    scalars(3)  = other.ENV.wgs84(2);
    scalars(4)  = other.ENV.omega_E;
    scalars(5)  = other.AER.Sref;
    scalars(6)  = other.MOT(1).mass_flow_rate;
    scalars(7)  = other.MOT(1).number_of_ignite_engine;
    scalars(8)  = other.MOT(1).vacuum_thrust;
    scalars(9)  = other.MOT(1).nozzle_exit_area;
    scalars(10) = other.MOT(2).mass_flow_rate;
    scalars(11) = other.MOT(2).number_of_ignite_engine;
    scalars(12) = other.MOT(2).vacuum_thrust;
    scalars(13) = other.MOT(2).nozzle_exit_area;
    scalars(14) = stage;
    scalars(15) = double(other.isignite);
    scalars(16) = other.phase;
    scalars(17) = other.GUI.launch_azimuth;
    scalars(18) = other.GUI.pitch_over_starting;
    scalars(19) = other.GUI.pitch(1);
    scalars(20) = other.GUI.pitch(2);
    scalars(21) = other.GUI.last_pitch;
    scalars(22) = other.GUI.last_yaw;
    scalars(23) = other.GUI.pitch_rate_transition;
    scalars(24) = other.GUI.transition_starting;
    scalars(25) = other.GUI.pitch_at_transition;
    scalars(26) = other.GUI.insertion_starting;
    scalars(27) = other.GUI.AoA_rate;
    scalars(28) = other.GUI.plane_controller(1);
    scalars(29) = other.GUI.plane_controller(2);
    scalars(30) = other.GUI.plane_controller(3);
    scalars(31) = other.MIS.target_orbital_inclination;
    scalars(32) = other.GUI.zkick;
    scalars(33) = other.MASS.Minert1;
    scalars(34) = other.MASS.Minert2;
    scalars(35) = other.MASS.MProp2;
    scalars(36) = other.MASS.Mfairing;
    scalars(37) = other.MASS.Mpayload;
    scalars(38) = other.MIS.apogee_altitude_target;

    InOl = other.GUI.InOl;

    env_alt = other.ENV.altitude(:);
    env_rho = other.ENV.atmospheric_density(:);
    env_c   = other.ENV.sound_speed(:);
    env_p   = other.ENV.ambient_pressure(:);

    aer_mach = other.AER.Mach(:);
    aer_aoa  = other.AER.AoA(:);
    aer_cd   = other.AER.Cd;
end
