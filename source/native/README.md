# native/ — kernel Fortran compilato per la simulazione TSTO

Porting Fortran di `eom.m`+`guidance.m` (fasi 1-6) e `phase_event.m`,
per eliminare l'overhead dell'interprete Octave sul loop caldo di
integrazione (`rk5.m`). Rif. `solver_project/CLAUDE.md` §11 Fase 5,
sessione "requisito 5 minuti": misurato 412.9x di speedup su
eom.m/guidance.m, 44.9x su phase_event.m.

## File

- `eom_core.f90` — kernel numerico (unico sorgente Fortran, contiene sia
  `eom_native_f` che `phase_event_native_f`).
- `eom_oct.cc`, `phase_event_oct.cc` — shim oct-file (marshaling Octave↔Fortran).
- `eom_native.oct`, `phase_event_native.oct` — **compilati, NON versionati**
  (vedi `.gitignore`): vanno ricostruiti dopo ogni clone.

## Build

Richiede il toolchain di sviluppo Octave (pacchetto Debian/Ubuntu
`octave-dev`, fornisce `mkoctfile`) e un compilatore Fortran (`gfortran`).

```bash
sudo apt install octave-dev gfortran   # se non gia' presenti
cd source/native
mkoctfile eom_oct.cc eom_core.f90 -o eom_native.oct
mkoctfile phase_event_oct.cc eom_core.f90 -o phase_event_native.oct
```

## Fallback automatico

`simulator.m` verifica `exist('eom_native','file')==3` /
`exist('phase_event_native','file')==3` prima di usare i kernel
compilati: se i file `.oct` non sono presenti (build non eseguita),
ricade automaticamente sulle funzioni Octave interpretate (`eom.m`,
`guidance.m`, `phase_event.m`) — corretto ma ~150-400x piu' lento.
Nessun errore se non si compila, solo piu' lento.
