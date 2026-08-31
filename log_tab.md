# Formato della tab.out
E' una tabella in formato testo a **larghezza di colonna fissa** (colonne allineate,
non semplice CSV) che riporta gli eventi più salienti della traiettoria.
- La prima riga è un header (un'etichetta per colonna).
- La seconda riga è un separatore di soli `=` lungo quanto l'intera riga di header,
  per staccare visivamente l'header dai dati.
- Dalla terza riga in poi, una riga per evento.

## Quali eventi sono tracciati
Per ciascuno di questi eventi è scritta un riga:
- ogni inizio e fine di fase
- ogni trigger che porta ad uscire dalla simulazione
- la pressione dinamica massima: `max q`
- il raggiungimento di Mach 1: `Mach 1`

## Nome evento
Massimo **12 caratteri**, maiuscolo, nessuno spazio (per restare leggibile nella
colonna a larghezza fissa). Convenzione usata da `write_log.m`:
- inizio/fine fase: `P<n>_START` / `P<n>_END`, `<n>` = numero di fase 1..6
  (es. `P3_START`, `P4_END`)
- pressione dinamica massima: `MAXQ`
- raggiungimento Mach 1: `MACH1`
- terminazione simulazione (messaggio esplicito da `simulator.m`, non dedotto):
  `END_CRASH` (quota=0), `END_APOGEE` (apogeo target raggiunto, fase 6),
  `END_PROP2` (propellente stadio 2 esaurito, fase 6)

## Lista delle variabili da riportare e precisione
- nome evento
- tempo [s], 3 decimali
- quota [km], 3 decimali
- massa [ton], 1 decimale
- velocita rel [m/s]. 1 decimale
- Mach [-], 1 decimale
- pitch [deg], 1 decimale
- yaw [deg], 1 decimale
- incidence [deg], 1 decimale
- apo_alt  [km], 1 decimale
- inclinazione [deg], 2 decimali
- delta_v [m/s], 1 decimale
