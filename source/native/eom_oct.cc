// eom_oct.cc -- shim oct-file per il kernel Fortran eom_core.f90.
//
// Motivazione e layout dati: vedi header di eom_core.f90 (rif. anche
// solver_project/CLAUDE.md S11 Fase 5, sessione "requisito 5 minuti").
// Nessuna logica fisica qui: solo marshaling degli argomenti Octave verso
// la firma bind(C) del sottoprogramma Fortran e ritorno di dy (8x1).
//
// Octave::Matrix/ColumnVector sono gia' in layout column-major (come
// Fortran): fortran_vec() restituisce il puntatore diretto ai dati senza
// bisogno di riordino/copia esplicita da parte nostra.

#include <octave/oct.h>

extern "C" {
  void eom_native_f (double t, const double *y, const double *s, const double *InOl,
                      int n_env, const double *env_alt, const double *env_rho,
                      const double *env_c, const double *env_p,
                      int n_mach, int n_aoa, const double *aer_mach, const double *aer_aoa,
                      const double *aer_cd, double *dy);
}

DEFUN_DLD (eom_native, args, ,
           "dy = eom_native(t, y, scalars, InOl, env_alt, env_rho, env_c, env_p, aer_mach, aer_aoa, aer_cd)\n\
Shim oct-file per il kernel Fortran eom_core.f90 (porting di eom.m+guidance.m,\n\
fasi 1-6). Vedi eom_core.f90 per il layout di 'scalars' (31 elementi) e\n\
build_eom_native_params.m per come costruire gli argomenti da 'other'.")
{
  octave_value_list retval;

  if (args.length () != 11)
    {
      error ("eom_native: attesi 11 argomenti (t, y, scalars, InOl, env_alt, env_rho, env_c, env_p, aer_mach, aer_aoa, aer_cd)");
      return retval;
    }

  double t = args(0).double_value ();

  ColumnVector y        = args(1).column_vector_value ();
  ColumnVector scalars  = args(2).column_vector_value ();
  Matrix InOl           = args(3).matrix_value ();
  ColumnVector env_alt  = args(4).column_vector_value ();
  ColumnVector env_rho  = args(5).column_vector_value ();
  ColumnVector env_c    = args(6).column_vector_value ();
  ColumnVector env_p    = args(7).column_vector_value ();
  ColumnVector aer_mach = args(8).column_vector_value ();
  ColumnVector aer_aoa  = args(9).column_vector_value ();
  Matrix aer_cd         = args(10).matrix_value ();

  if (y.numel () != 8 || scalars.numel () < 31
      || InOl.rows () != 3 || InOl.cols () != 3)
    {
      // scalars puo' essere piu' lungo di 31 (condiviso con
      // phase_event_native, che ne usa 38: build_eom_native_params.m
      // costruisce un unico vettore per entrambi) -- eom_native legge
      // solo i primi 31, nessun problema a riceverne di piu'.
      error ("eom_native: dimensioni non valide su y (attesi 8), scalars (attesi >=31) o InOl (attesi 3x3)");
      return retval;
    }

  octave_idx_type n_env  = env_alt.numel ();
  octave_idx_type n_mach = aer_mach.numel ();
  octave_idx_type n_aoa  = aer_aoa.numel ();

  if (env_rho.numel () != n_env || env_c.numel () != n_env || env_p.numel () != n_env)
    {
      error ("eom_native: env_rho/env_c/env_p devono avere la stessa lunghezza di env_alt");
      return retval;
    }
  if (aer_cd.rows () != n_mach || aer_cd.cols () != n_aoa)
    {
      error ("eom_native: aer_cd deve avere dimensione [n_mach x n_aoa]");
      return retval;
    }

  ColumnVector dy (8);

  eom_native_f (t, y.fortran_vec (), scalars.fortran_vec (), InOl.fortran_vec (),
                static_cast<int> (n_env), env_alt.fortran_vec (), env_rho.fortran_vec (),
                env_c.fortran_vec (), env_p.fortran_vec (),
                static_cast<int> (n_mach), static_cast<int> (n_aoa),
                aer_mach.fortran_vec (), aer_aoa.fortran_vec (), aer_cd.fortran_vec (),
                dy.fortran_vec ());

  retval(0) = dy;
  return retval;
}
