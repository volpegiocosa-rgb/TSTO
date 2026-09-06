! eom_core.f90 -- porting Fortran del blocco derivata di stato TSTO
! (eom.m + guidance.m casi 1-6 + funzioni ausiliarie chiamate).
!
! Motivazione (rif. solver_project/CLAUDE.md S11 Fase 5, sessione
! "requisito 5 minuti"): eom.m/guidance.m sono chiamate 7 volte per punto
! RK5 accettato (k1..k6 di rk5.m + 1 di kinematic_step.m); misurato in
! sessione: ~5.7 ms/chiamata su Octave interpretato, per un contenuto di
! calcolo minimo -- costo quasi certamente dominato dall'overhead
! dell'interprete (dispatch di funzione, accesso a struct annidate), non
! da lavoro numerico reale. Questo file replica ESATTAMENTE le formule
! della sorgente Octave (stesso ordine di operazioni, stesse clamp) per
! eliminare quell'overhead.
!
! Scope: SOLO le fasi 1-6 (le uniche integrate da rk5.m -- le fasi 7-8
! sono istantanee, mai chiamate nel loop ODE, CLAUDE.md TSTO S5).
! phase_event.m NON e' ancora portato (chiamato 1 volta per punto
! accettato, non 7 come eom.m: priorita' minore, da riconsiderare solo se
! resta un collo di bottiglia dopo aver misurato lo speedup di questo file).
!
! Layout del vettore scalars(31) (costruito una volta per fase da
! build_eom_native_params.m, NON ad ogni chiamata):
!   1  mu                       17 launch_azimuth
!   2  Req (wgs84(1))           18 pitch_over_starting
!   3  f   (wgs84(2))           19 pitch(1)  [pitch_c1]
!   4  omega_E                  20 pitch(2)  [pitch_c2]
!   5  Sref                     21 last_pitch
!   6  mass_flow_rate stadio1   22 last_yaw
!   7  number_of_ignite_engine1 23 pitch_rate_transition
!   8  vacuum_thrust stadio1    24 transition_starting
!   9  nozzle_exit_area stadio1 25 pitch_at_transition
!   10 mass_flow_rate stadio2   26 insertion_starting
!   11 number_of_ignite_engine2 27 AoA_rate
!   12 vacuum_thrust stadio2    28 plane_controller(1) kp
!   13 nozzle_exit_area stadio2 29 plane_controller(2) kd
!   14 active_stage (1|2)       30 plane_controller(3) ki
!   15 isignite (0|1)           31 target_orbital_inclination
!   16 phase (1..6)
!
! Tabelle ENV (env_alt/env_rho/env_c/env_p, lunghezza n_env, ordinate
! crescenti -- verificato issorted() in sessione) e AER (aer_mach/aer_aoa
! crescenti, aer_cd(n_mach,n_aoa): righe=Mach, colonne=AoA, layout
! column-major nativo Octave == column-major nativo Fortran, passate
! dallo shim C++ con Matrix::fortran_vec(), nessun riordino necessario).

module eom_core_mod
  use, intrinsic :: iso_c_binding
  implicit none
  private
  public :: eom_native_f
  public :: phase_event_native_f

  real(c_double), parameter :: PI_  = 3.14159265358979323846d0
  real(c_double), parameter :: HALFPI_ = 1.57079632679489661923d0

contains

  pure function norm3(v) result(n)
    real(c_double), intent(in) :: v(3)
    real(c_double) :: n
    n = sqrt(v(1)*v(1) + v(2)*v(2) + v(3)*v(3))
  end function norm3

  pure function vers3(v) result(u)
    real(c_double), intent(in) :: v(3)
    real(c_double) :: u(3), n
    n = norm3(v)
    if (n > 1.0d-12) then
      u = v / n
    else
      u = 0.0d0
    end if
  end function vers3

  pure function cross3(a, b) result(c)
    real(c_double), intent(in) :: a(3), b(3)
    real(c_double) :: c(3)
    c(1) = a(2)*b(3) - a(3)*b(2)
    c(2) = a(3)*b(1) - a(1)*b(3)
    c(3) = a(1)*b(2) - a(2)*b(1)
  end function cross3

  pure function dot3(a, b) result(d)
    real(c_double), intent(in) :: a(3), b(3)
    real(c_double) :: d
    d = a(1)*b(1) + a(2)*b(2) + a(3)*b(3)
  end function dot3

  pure function matvec3(M, v) result(r)
    real(c_double), intent(in) :: M(3,3), v(3)
    real(c_double) :: r(3)
    r = matmul(M, v)
  end function matvec3

  pure function mattvec3(M, v) result(r)
    real(c_double), intent(in) :: M(3,3), v(3)
    real(c_double) :: r(3)
    r = matmul(transpose(M), v)
  end function mattvec3

  pure function setOl_f(pitch, yaw) result(u)
    real(c_double), intent(in) :: pitch, yaw
    real(c_double) :: u(3)
    u(1) = cos(pitch) * sin(yaw)
    u(2) = cos(pitch) * cos(yaw)
    u(3) = sin(pitch)
  end function setOl_f

  subroutine vect2angleOl_f(uOl, pitch, yaw)
    real(c_double), intent(in)  :: uOl(3)
    real(c_double), intent(out) :: pitch, yaw
    real(c_double) :: z
    z = min(1.0d0, max(-1.0d0, uOl(3)))
    pitch = asin(z)
    yaw   = atan2(uOl(1), uOl(2))
  end subroutine vect2angleOl_f

  subroutine eval_aero_angle_f(relative_speed, pos, u_ref, incidence, sideslip)
    real(c_double), intent(in)  :: relative_speed(3), pos(3), u_ref(3)
    real(c_double), intent(out) :: incidence, sideslip
    real(c_double) :: x_hat(3), z_hat(3), y_hat(3), u1, u2, u3
    x_hat = vers3(relative_speed)
    z_hat = vers3(cross3(pos, relative_speed))
    y_hat = cross3(z_hat, x_hat)
    u1 = dot3(x_hat, u_ref)
    u2 = dot3(y_hat, u_ref)
    u3 = dot3(z_hat, u_ref)
    incidence = atan2(u2, u1)
    sideslip  = atan2(u3, u1)
  end subroutine eval_aero_angle_f

  pure function eval_AoA_f(u, relative_speed) result(AoA)
    real(c_double), intent(in) :: u(3), relative_speed(3)
    real(c_double) :: AoA, vr, cosAoA
    vr = norm3(relative_speed)
    if (vr > 1.0d-3) then
      cosAoA = dot3(u, relative_speed) / vr
      cosAoA = min(1.0d0, max(-1.0d0, cosAoA))
      AoA = acos(cosAoA)
    else
      AoA = 0.0d0
    end if
  end function eval_AoA_f

  pure function eval_relative_speed_f(pos, vel, omega_E) result(rs)
    real(c_double), intent(in) :: pos(3), vel(3), omega_E
    real(c_double) :: rs(3), omega_vec(3)
    omega_vec = (/ 0.0d0, 0.0d0, omega_E /)
    rs = vel - cross3(omega_vec, pos)
  end function eval_relative_speed_f

  pure function cart2geo_alt(pos, Req, f) result(alt)
    real(c_double), intent(in) :: pos(3), Req, f
    real(c_double) :: alt
    real(c_double) :: e2, x, y, z, p, lat, lat_new, sinlat, N
    integer :: iter
    e2 = f * (2.0d0 - f)
    x = pos(1); y = pos(2); z = pos(3)
    p = sqrt(x*x + y*y)
    if (p < 1.0d-9) then
      alt = abs(z) - Req * sqrt(1.0d0 - e2)
      return
    end if
    lat = atan2(z, p*(1.0d0 - e2))
    do iter = 1, 10
      sinlat = sin(lat)
      N   = Req / sqrt(1.0d0 - e2*sinlat*sinlat)
      alt = p/cos(lat) - N
      lat_new = atan2(z, p*(1.0d0 - e2*N/(N+alt)))
      if (abs(lat_new - lat) < 1.0d-12) then
        lat = lat_new
        exit
      end if
      lat = lat_new
    end do
    sinlat = sin(lat)
    N   = Req / sqrt(1.0d0 - e2*sinlat*sinlat)
    alt = p/cos(lat) - N
  end function cart2geo_alt

  pure function interp1_lin(x, y, n, xq) result(yq)
    integer(c_int), intent(in) :: n
    real(c_double), intent(in) :: x(n), y(n), xq
    real(c_double) :: yq, t
    integer :: i
    if (n == 1) then
      yq = y(1)
      return
    end if
    i = n - 1
    do i = 1, n-1
      if (xq <= x(i+1)) exit
    end do
    if (i > n-1) i = n-1
    t = (xq - x(i)) / (x(i+1) - x(i))
    yq = y(i) + t*(y(i+1) - y(i))
  end function interp1_lin

  pure function interp2_lin(aoa, mach, cd, n_mach, n_aoa, aoaq, machq) result(cdq)
    integer(c_int), intent(in) :: n_mach, n_aoa
    real(c_double), intent(in) :: aoa(n_aoa), mach(n_mach), cd(n_mach, n_aoa)
    real(c_double), intent(in) :: aoaq, machq
    real(c_double) :: cdq
    integer :: ix, iy
    real(c_double) :: tx, ty, z00, z01, z10, z11

    if (n_aoa == 1) then
      ix = 1; tx = 0.0d0
    else
      do ix = 1, n_aoa-1
        if (aoaq <= aoa(ix+1)) exit
      end do
      if (ix > n_aoa-1) ix = n_aoa-1
      tx = (aoaq - aoa(ix)) / (aoa(ix+1) - aoa(ix))
    end if

    if (n_mach == 1) then
      iy = 1; ty = 0.0d0
    else
      do iy = 1, n_mach-1
        if (machq <= mach(iy+1)) exit
      end do
      if (iy > n_mach-1) iy = n_mach-1
      ty = (machq - mach(iy)) / (mach(iy+1) - mach(iy))
    end if

    z00 = cd(iy,   ix)
    z01 = cd(iy,   ix+1)
    z10 = cd(iy+1, ix)
    z11 = cd(iy+1, ix+1)

    cdq = (1.0d0-ty)*((1.0d0-tx)*z00 + tx*z01) + ty*((1.0d0-tx)*z10 + tx*z11)
  end function interp2_lin

  subroutine guidance_native(s, InOl, t, pos, vel, relative_speed, phase, uIn)
    real(c_double), intent(in)  :: s(31), InOl(3,3), t, pos(3), vel(3), relative_speed(3)
    integer(c_int), intent(in)  :: phase
    real(c_double), intent(out) :: uIn(3)

    real(c_double) :: uOl(3), dt, pitch, yaw, pitch_rate, incidence, sideslip
    real(c_double) :: u_ref(3), uOl_rel(3), rel_ol(3), pitch_rel, yaw_rel
    real(c_double) :: h(3), nh, actual_incl, err, yaw_corr, AoA_cmd

    select case (phase)
    case (1)
      uOl = (/ 0.0d0, 0.0d0, 1.0d0 /)
      uIn = matvec3(InOl, uOl)

    case (2)
      dt    = t - s(18)
      pitch = HALFPI_ + s(19)*dt*dt + s(20)*dt
      yaw   = s(17)
      uOl   = setOl_f(pitch, yaw)
      uIn   = matvec3(InOl, uOl)

    case (3)
      u_ref = matvec3(InOl, setOl_f(s(21), s(22)))
      call eval_aero_angle_f(relative_speed, pos, u_ref, incidence, sideslip)
      if (incidence > 0.0d0) then
        pitch_rate = -s(23)
      else if (incidence < 0.0d0) then
        pitch_rate = s(23)
      else
        pitch_rate = 0.0d0
      end if
      dt    = t - s(24)
      pitch = pitch_rate*dt + s(25)
      yaw   = s(17)
      uOl   = setOl_f(pitch, yaw)
      uIn   = matvec3(InOl, uOl)

    case (4, 5)
      rel_ol  = mattvec3(InOl, relative_speed)
      uOl_rel = vers3(rel_ol)
      call vect2angleOl_f(uOl_rel, pitch, yaw)
      yaw = s(17)
      uOl = setOl_f(pitch, yaw)
      uIn = matvec3(InOl, uOl)

    case (6)
      rel_ol  = mattvec3(InOl, relative_speed)
      uOl_rel = vers3(rel_ol)
      call vect2angleOl_f(uOl_rel, pitch_rel, yaw_rel)

      h  = cross3(pos, vel)
      nh = norm3(h)
      if (nh > 1.0d-9) then
        actual_incl = acos(min(1.0d0, max(-1.0d0, h(3)/nh)))
      else
        actual_incl = 0.0d0
      end if
      err      = s(31) - actual_incl
      yaw_corr = (s(28) + s(29) + s(30)) * err
      yaw      = yaw_rel + yaw_corr

      dt      = t - s(26)
      AoA_cmd = s(27) * dt
      pitch   = pitch_rel + AoA_cmd

      uOl = setOl_f(pitch, yaw)
      uIn = matvec3(InOl, uOl)

    case default
      uIn = 0.0d0
    end select
  end subroutine guidance_native

  subroutine eom_native_f(t, y, s, InOl, n_env, env_alt, env_rho, env_c, env_p, &
                           n_mach, n_aoa, aer_mach, aer_aoa, aer_cd, dy) &
                           bind(C, name="eom_native_f")
    real(c_double), intent(in), value :: t
    real(c_double), intent(in) :: y(8)
    real(c_double), intent(in) :: s(31)
    real(c_double), intent(in) :: InOl(3,3)
    integer(c_int), intent(in), value :: n_env
    real(c_double), intent(in) :: env_alt(n_env), env_rho(n_env), env_c(n_env), env_p(n_env)
    integer(c_int), intent(in), value :: n_mach, n_aoa
    real(c_double), intent(in) :: aer_mach(n_mach), aer_aoa(n_aoa)
    real(c_double), intent(in) :: aer_cd(n_mach, n_aoa)
    real(c_double), intent(out) :: dy(8)

    real(c_double) :: mu, Req, f, omega_E, Sref
    real(c_double) :: mfr(2), nie(2), vt(2), nea(2)
    integer :: active_stage, phase, st
    logical :: isignite
    real(c_double) :: pos(3), vel(3), mass
    real(c_double) :: rm, g(3)
    real(c_double) :: alt, alt_q, rho, sound_speed, p_amb
    real(c_double) :: relative_speed(3), vrel, Mach
    real(c_double) :: uIn(3), AoA
    real(c_double) :: Cd, Mach_q, AoA_q, AoA_deg
    real(c_double) :: Drag(3), rv(3), qdyn
    real(c_double) :: mass_flow_rate, thrust, vacuum_thrust
    real(c_double) :: vectorized_thrust(3), not_grav(3)

    mu = s(1); Req = s(2); f = s(3); omega_E = s(4); Sref = s(5)
    mfr(1) = s(6);  nie(1) = s(7);  vt(1) = s(8);  nea(1) = s(9)
    mfr(2) = s(10); nie(2) = s(11); vt(2) = s(12); nea(2) = s(13)
    active_stage = nint(s(14))
    isignite     = (s(15) /= 0.0d0)
    phase        = nint(s(16))

    pos  = y(1:3)
    vel  = y(4:6)
    mass = y(7)

    rm = norm3(pos)
    g  = -mu/rm**3 * pos

    alt   = cart2geo_alt(pos, Req, f)
    alt_q = min(max(alt, env_alt(1)), env_alt(n_env))
    rho         = interp1_lin(env_alt, env_rho, n_env, alt_q)
    sound_speed = interp1_lin(env_alt, env_c,   n_env, alt_q)
    p_amb       = interp1_lin(env_alt, env_p,   n_env, alt_q)

    relative_speed = eval_relative_speed_f(pos, vel, omega_E)
    vrel = norm3(relative_speed)
    Mach = vrel / sound_speed

    call guidance_native(s, InOl, t, pos, vel, relative_speed, phase, uIn)
    AoA = eval_AoA_f(uIn, relative_speed)

    if (active_stage == 1) then
      Mach_q  = min(max(Mach, aer_mach(1)), aer_mach(n_mach))
      AoA_deg = AoA * 180.0d0 / PI_
      AoA_q   = min(max(AoA_deg, aer_aoa(1)), aer_aoa(n_aoa))
      Cd = interp2_lin(aer_aoa, aer_mach, aer_cd, n_mach, n_aoa, AoA_q, Mach_q)
    else
      Cd = 0.0d0
    end if

    if (vrel > 0.1d0) then
      rv   = vers3(relative_speed)
      qdyn = 0.5d0 * rho * Cd * Sref * vrel*vrel
      Drag = qdyn * rv
    else
      Drag = 0.0d0
    end if

    st = active_stage
    mass_flow_rate = 0.0d0
    thrust         = 0.0d0
    if (isignite) then
      mass_flow_rate = mfr(st) * nie(st)
      vacuum_thrust  = vt(st) * nie(st)
      thrust         = vacuum_thrust - p_amb * nea(st) * nie(st)
    end if

    vectorized_thrust = thrust * uIn
    not_grav = vectorized_thrust/mass - Drag/mass

    dy(1:3) = vel
    dy(4:6) = not_grav + g
    dy(7)   = -mass_flow_rate
    dy(8)   = norm3(not_grav)
  end subroutine eom_native_f

  pure function eval_apogee_altitude_f(pos, vel, mu, Req) result(apogee_altitude)
    ! Porting di eval_apogee_altitude.m (quota di apogeo osculante,
    ! approssimazione sferica -- usata da phase_event.m case 6).
    real(c_double), intent(in) :: pos(3), vel(3), mu, Req
    real(c_double) :: apogee_altitude
    real(c_double) :: r, v, energy, a, h(3), e_vec(3), e, apogee_radius
    r = norm3(pos)
    v = norm3(vel)
    energy = v*v/2.0d0 - mu/r
    a = -mu / (2.0d0*energy)
    h = cross3(pos, vel)
    e_vec = cross3(vel, h)/mu - pos/r
    e = norm3(e_vec)
    apogee_radius = a * (1.0d0 + e)
    apogee_altitude = apogee_radius - Req
  end function eval_apogee_altitude_f

  subroutine phase_event_native_f(t, y, s, InOl, phase, val, isterminal, direction, n_out) &
                                   bind(C, name="phase_event_native_f")
    ! Porting di phase_event.m. Layout di 's': vedi build_eom_native_params.m
    ! (1-31 condivisi con eom_native_f, 32-38 aggiunti qui). val/isterminal/
    ! direction sono sempre dimensionati 3 (il massimo, fasi 1/2/3/6): per
    ! le fasi 4/5 (2 componenti) solo i primi n_out elementi sono validi --
    ! lo shim oct-file tronca a n_out prima di restituire a Octave, stessa
    ! semantica a lunghezza variabile dell'originale.
    real(c_double), intent(in), value :: t
    real(c_double), intent(in) :: y(8)
    real(c_double), intent(in) :: s(38)
    real(c_double), intent(in) :: InOl(3,3)
    integer(c_int), intent(in), value :: phase
    real(c_double), intent(out) :: val(3)
    real(c_double), intent(out) :: isterminal(3)
    real(c_double), intent(out) :: direction(3)
    integer(c_int), intent(out) :: n_out

    real(c_double) :: pos(3), vel(3), mass
    real(c_double) :: mu, Req, f, omega_E
    real(c_double) :: last_pitch, last_yaw, transition_starting, insertion_starting
    real(c_double) :: zkick, Minert1, Minert2, MProp2, Mfairing, Mpayload, apogee_target
    real(c_double) :: altitude, altitude_zero_ev
    real(c_double) :: dead_mass_stage1, dead_mass_stage2
    real(c_double) :: relative_speed(3), u_ref(3), incidence, sideslip
    real(c_double) :: apogee_altitude

    mu = s(1); Req = s(2); f = s(3); omega_E = s(4)
    last_pitch = s(21); last_yaw = s(22)
    transition_starting = s(24)
    insertion_starting  = s(26)
    zkick    = s(32)
    Minert1  = s(33)
    Minert2  = s(34)
    MProp2   = s(35)
    Mfairing = s(36)
    Mpayload = s(37)
    apogee_target = s(38)

    pos  = y(1:3)
    vel  = y(4:6)
    mass = y(7)

    altitude = cart2geo_alt(pos, Req, f)
    altitude_zero_ev = altitude

    dead_mass_stage1 = Minert1 + Minert2 + MProp2 + Mfairing + Mpayload
    dead_mass_stage2 = Minert2 + Mpayload

    val = 0.0d0; isterminal = 0.0d0; direction = 0.0d0

    select case (phase)
    case (1)
      val(1) = altitude - zkick
      val(2) = altitude_zero_ev
      val(3) = mass - dead_mass_stage1
      isterminal(1:3) = 1.0d0
      direction = (/ 1.0d0, -1.0d0, -1.0d0 /)
      n_out = 3

    case (2)
      val(1) = t - transition_starting
      val(2) = altitude_zero_ev
      val(3) = mass - dead_mass_stage1
      isterminal(1:3) = 1.0d0
      direction = (/ 1.0d0, -1.0d0, -1.0d0 /)
      n_out = 3

    case (3)
      relative_speed = eval_relative_speed_f(pos, vel, omega_E)
      u_ref = matvec3(InOl, setOl_f(last_pitch, last_yaw))
      call eval_aero_angle_f(relative_speed, pos, u_ref, incidence, sideslip)
      val(1) = incidence
      val(2) = altitude_zero_ev
      val(3) = mass - dead_mass_stage1
      isterminal(1:3) = 1.0d0
      direction = (/ 0.0d0, -1.0d0, -1.0d0 /)
      n_out = 3

    case (4)
      val(1) = mass - dead_mass_stage1
      val(2) = altitude_zero_ev
      isterminal(1:2) = 1.0d0
      direction(1) = -1.0d0
      direction(2) = -1.0d0
      n_out = 2

    case (5)
      val(1) = t - insertion_starting
      val(2) = altitude_zero_ev
      isterminal(1:2) = 1.0d0
      direction(1) = 1.0d0
      direction(2) = -1.0d0
      n_out = 2

    case (6)
      apogee_altitude = eval_apogee_altitude_f(pos, vel, mu, Req)
      val(1) = apogee_altitude - apogee_target
      val(2) = altitude_zero_ev
      val(3) = mass - dead_mass_stage2
      isterminal(1:3) = 1.0d0
      direction = (/ 1.0d0, -1.0d0, -1.0d0 /)
      n_out = 3

    case default
      n_out = 0
    end select
  end subroutine phase_event_native_f

end module eom_core_mod
