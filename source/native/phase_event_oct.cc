// phase_event_oct.cc -- shim oct-file per phase_event_native_f
// (eom_core.f90, porting di phase_event.m). Vedi header di eom_core.f90
// per il layout di 'scalars' (38 elementi) e build_eom_native_params.m
// per come costruirlo da 'other'.
//
// Rif. CLAUDE.md solver_project S11 Fase 5, sessione "requisito 5
// minuti": phase_event.m e' chiamato 1 volta per punto RK5 accettato
// (non 7 come eom.m/guidance.m) ma, dopo aver portato eom_native, e'
// rimasto il principale collo di bottiglia residuo (misurato: 7.47s ->
// 1.32s end-to-end con solo eom_native, non ancora abbastanza per il
// requisito di 5 minuti sul caso reale).

#include <octave/oct.h>

extern "C" {
  void phase_event_native_f (double t, const double *y, const double *s, const double *InOl,
                              int phase, double *val, double *isterminal, double *direction,
                              int *n_out);
}

DEFUN_DLD (phase_event_native, args, ,
           "[value, isterminal, direction] = phase_event_native(t, y, scalars, InOl, phase)\n\
Shim oct-file per il kernel Fortran phase_event_native_f (porting di\n\
phase_event.m). Vedi eom_core.f90 per il layout di 'scalars' (38 elementi).")
{
  octave_value_list retval;

  if (args.length () != 5)
    {
      error ("phase_event_native: attesi 5 argomenti (t, y, scalars, InOl, phase)");
      return retval;
    }

  double t = args(0).double_value ();
  ColumnVector y       = args(1).column_vector_value ();
  ColumnVector scalars = args(2).column_vector_value ();
  Matrix InOl          = args(3).matrix_value ();
  int phase            = static_cast<int> (args(4).double_value ());

  if (y.numel () != 8 || scalars.numel () < 38
      || InOl.rows () != 3 || InOl.cols () != 3)
    {
      error ("phase_event_native: dimensioni non valide su y (attesi 8), scalars (attesi >=38) o InOl (attesi 3x3)");
      return retval;
    }

  double val[3] = {0.0, 0.0, 0.0};
  double isterminal[3] = {0.0, 0.0, 0.0};
  double direction[3] = {0.0, 0.0, 0.0};
  int n_out = 0;

  phase_event_native_f (t, y.fortran_vec (), scalars.fortran_vec (), InOl.fortran_vec (),
                         phase, val, isterminal, direction, &n_out);

  if (n_out <= 0 || n_out > 3)
    {
      error ("phase_event_native: fase non valida o n_out inatteso (%d)", n_out);
      return retval;
    }

  ColumnVector V (n_out), IT (n_out), D (n_out);
  for (int i = 0; i < n_out; i++)
    {
      V(i)  = val[i];
      IT(i) = isterminal[i];
      D(i)  = direction[i];
    }

  retval(0) = V;
  retval(1) = IT;
  retval(2) = D;
  return retval;
}
