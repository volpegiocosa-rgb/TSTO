Sei l'assistente di sviluppo di questo repository. PRIMA di scrivere codice:

1. Leggi CLAUDE.md e interface_specification.md per intero. Rispetta TUTTE
   le convenzioni (Octave compatibile Matlab 2025A, no oggetti, no variabili
   globali, no endif/endfor/#, struct annidate max 4 livelli, NaN-check).
2. Leggi i file gia' presenti in /source: interface.m, simulator.m, eom.m,
   guidance.m. Non riscriverli da zero: completa i PLACEHOLDER/TODO esistenti.

POI, prima di toccare il codice, presentami un PIANO:
- elenco dei file .m da creare/completare e in che ordine;
- per ogni file: firma, input/output, funzioni ausiliarie richiamate;
- come intendi risolvere i TODO aperti (griglia Cd(Mach,AoA) via interp2,
  GUI.InOl, gestione event/trigger di fase, separazione 1' stadio a fine fase 4,
  init_state, eval_liftoff_propellant).
Fermati e aspetta la mia conferma sul piano.

IMPORTANTE - In caso di dubbi, CHIEDI:
- Se qualcosa in CLAUDE.md o nella specifica non e' chiaro, ambiguo o
  contraddittorio, FERMATI e fammi una domanda mirata invece di assumere.
- Non colmare le lacune con ipotesi silenziose: preferisco una domanda in
  piu' a una scelta sbagliata data per scontata.
- Va bene procedere solo quando hai capito con certezza cosa serve.

Dati di input e caso di test di riferimento:
- Il dataset di input primario e' in /input/reference_LV (un lanciatore a 2
  stadi a liquido Falcon 9-like, dedotto da specifiche pubbliche). Contiene i
  CSV: LV.csv, ENV.csv, GUID.csv, GUIDANCE_VARS.csv, MIS.csv, atmosphere.csv,
  aero_ascent.csv, piu' un README.md che ne spiega formato, unita' e fonti.
- I CSV seguono la CONVENZIONE INTERNA DEL CODICE (ENV/AER/MOT/GUI/MIS), NON le
  struct del documento di interfaccia: interface_specification.md va usato SOLO
  per gli output (RES).
- interface.m gia' legge questa cartella: la firma e' interface(input_dir) e
  simulator.m la chiama con other = interface(config.input_dir), dove
  config.input_dir punta a /input/reference_LV.
- Usa reference_LV come caso di test PRIMARIO (ground truth: apogeo target,
  tempo di volo, ProfiloDV). Verifica coerenza fisica: burn stadio 1 ~142 s,
  T/W al liftoff ~1.4. Il synthetic test di CLAUDE.md §10 resta come smoke test
  secondario. Se un dato ti sembra incoerente col README, segnalalo prima di
  procedere.

Regole di lavoro:
- Procedi in modo INCREMENTALE, un modulo alla volta, e dopo ognuno esegui
  il lint a due passi descritto in CLAUDE.md §2.
- Non inventare campi non previsti dalla specifica. Se un dato manca,
  segnalalo e proponi un default ragionevole documentato, non nascosto.
- Rispetta la tabella unica fase <-> theGuidFlag <-> case guidance.m <->
  active_stage (CLAUDE.md §5). Il codice guidance.m e' la sorgente di verita'.
- Alla fine gira il caso reference_LV end-to-end e verifica la
  Definition of Done §9.

Nota sui moduli mancanti (mia IPOTESI, da confermare - non prenderla come
vincolo): oltre ai file gia' presenti, credo servano moduli come
create_output.m, plotter.m, e alcune funzioni ausiliarie richiamate dal
codice (es. init_state, phase_event/get_phase_event, eval_liftoff_propellant,
cart2geo, eval_relative_speed, eval_AoA, eval_aerodynamic_angle,
eval_inclination, eval_fpa, setOl, vect2angleOl, vers, PID_actuation).
Verifica tu quali servono davvero leggendo il codice, correggi o integra
questo elenco nel piano, e segnalami eventuali funzioni che ti aspettavi
di trovare ma mancano.

Comincia leggendo i file e proponendo il piano.
