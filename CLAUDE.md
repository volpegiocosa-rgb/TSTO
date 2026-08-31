# CLAUDE.md

Questo file fornisce istruzioni a Claude Code (claude.ai/code) per lavorare su questo repository.
Leggilo all'inizio di ogni sessione e rispetta **tutte** le convenzioni indicate.

---

## 1. Overview del progetto

Simulatore di un **veicolo di lancio a due stadi con propulsione a liquido**.

- Il **primo stadio** può avere più di un motore.
- Il veicolo è dotato di una **copertura (fairing)** che protegge il satellite (payload) al suo interno.
- Il simulatore integra le **equazioni del moto** (`/source/eom.m`) tramite un risolutore di equazioni differenziali a **passo variabile con controllo dell'errore** (es. `ode45`).
- Ogni **fase di volo** viene interrotta dalla verifica di uno degli **event** monitorati.

---

## 2. Comandi

> ⚠️ Sezione critica: da compilare con i comandi reali. Non inventare comandi di avvio.

### Avvio della simulazione
`simulator.m` è una **function-file** (`RES = simulator(config)`): Octave la definisce ma
NON la invoca automaticamente se passata da riga di comando (verificato: `octave --no-gui
source/simulator.m` non esegue nulla). Il punto di ingresso CLI reale è `source/run_simulator.m`
(script, dataset di default `input/reference_LV`, scrive l'output in `/output`):
```
octave --no-gui source/run_simulator.m
```
Per un dataset diverso, chiamare `simulator.m` esplicitamente via `--eval`:
```
octave --no-gui --eval "addpath('source'); config.input_dir='input/<dataset>'; config.silent=true; RES=simulator(config); write_output_csv(RES,'output');"
```

### Esecuzione dei test
Il caso di test **non è fornito**: deve essere **definito da Claude stesso** (vedi §10).
Convenzione di lancio del test una volta costruito:
```
octave --no-gui --eval "addpath('source'); config.input_dir='input/validation_test'; config.silent=true; RES=simulator(config); write_output_csv(RES,'output');"
```

### Verifica / lint di un singolo file
Octave non ha un linter integrato: si usa un controllo in **due passi** (sostituire `<FILE>`, es. `source/eom.m`).

**Passo 1 — controllo di sintassi (senza eseguire il corpo della function):**
```
octave --no-gui -qf --eval "addpath('source'); nargin('<FILE_SENZA_ESTENSIONE>')"
```
`nargin('nomefunzione')` forza Octave a **parsare** il file: se c'è un errore di sintassi lo segnala, altrimenti stampa il numero di argomenti di ingresso, **senza** eseguire la logica. Es.: `nargin('eom')`.

**Passo 2 — costrutti non compatibili con Matlab 2025A:**
```
grep -nE 'endif|endfor|endwhile|endswitch|end_try_catch|^\s*#' source/<FILE>
```
Il comando deve restituire **zero righe**. Qualsiasi match indica sintassi Octave-only (`endif`/`endfor`/... o commenti `#`) da correggere in `end` / `%`.

### Input / Output
- Input:  `/input`   (file **CSV delimitati da spazio**)
- Output: `/output`  (file **CSV delimitati da spazio**)

---

## 3. Architettura e struttura delle cartelle

```
/source     -> tutti i file .m del codice
/input      -> file di input (CSV delimitati da spazio)
/output     -> file di output del codice (CSV delimitati da spazio)
/docs       -> file markdown con la documentazione
```

### Moduli principali e responsabilità
- **`interface.m`** — chiamato **come primo passo** da `simulator.m`. Esegue il **mapping** delle variabili dalla convenzione della specifica (`LV / GUID / TARG / CONSTR`, vedi `interface_specification.md`) alla convenzione interna del codice (`ENV / AER / MOT / GUI / MIS`, raccolte in `other`). Isola il codice di calcolo dai nomi del docx.
- **`simulator.m`** — gestisce una fase dopo l'altra (è il *fasatore* di `eom.m`) ed è il **punto di interfaccia con l'utente**. Chiama `interface.m` prima di iniziare l'integrazione. Aggiorna `GUI.active_stage` (da 1 a 2 a fine fase 4, alla separazione del 1° stadio per esaurimento propellente).
- **`eom.m`** — core della simulazione; contiene le equazioni del moto. Chiamato da `simulator.m`.
- **`guidance.m`** — logica di guida; chiamata da `eom.m`.
- **`create_output.m`** — dagli output di `eom.m` all'interno di `simulator.m` crea **gli output** previsti dalla **specifica di interfaccia**.
- **`plotter.m`** — chiamato da `simulator.m`, crea i grafici se non è silenziato.

Flusso: `simulator.m` → `interface.m` (mapping input) → `eom.m` → `guidance.m` → `create_output.m` → `plotter.m`.

---

## 4. Convenzioni di codice (coding conventions)

### Linguaggio e norme
- **Octave**. Evitare sintassi non compatibile con **Matlab 2025A** (es. `endif`, `endfor`, `#` per i commenti → usare `%`).
- Evitare quando possibile il download di toolbox: preferire **function homemade**.
- Usare **solo script e function**. **Non** usare oggetti.
- **Nessuna variabile globale**: il codice deve essere pronto per l'esecuzione **in parallelo**.
- Nomi delle variabili **autoesplicativi**.
- Annidare le variabili in **struct fino a 4 livelli**.
- Controllare i potenziali punti di **`NaN` o numeri immaginari**.

---

## 5. Modello fisico — Fasi di volo

Il raggiungimento di un **trigger** fa passare alla fase successiva.

> **Fase 0 (Lift-off) NON è simulata**: viene calcolata in `simulator.m` per determinare quanto
> propellente deve essere bruciato per raggiungere il trigger di lift-off (il risultato può anche
> essere `t = 0 s`). L'integrazione delle EOM parte quindi dalla fase 1.

| N. | Nome della fase | Trigger | Condizione verificata tramite event | Fase propulsa |
|----|-----------------|---------|-------------------------------------|---------------|
| 0 | Lift-off | Quando l'accelerazione non gravitazionale eguaglia l'accelerazione di gravità locale | No (**non simulata**: calcolata all'inizio in `simulator.m`) | Sì |
| 1 | Vertical-rise | Raggiungimento della quota prevista in `GUI.zkick` | Sì | Sì |
| 2 | Pitch over | Durata assegnata | No | Sì |
| 3 | Transizione al gravity turn | Raggiungimento `incidence = 0` | Sì | Sì |
| 4 | Gravity turn | Fino a condizioni di separazione primo stadio (massa propellente usabile esaurita) | Sì | Sì |
| 5 | Coasting | Durata assegnata | No | No |
| 6 | Insertion in transfer orbit | Raggiungimento della quota di apogeo uguale a `MIS.apogee_altitude_target` | Sì | Sì |

> **Nota sulla colonna "Condizione verificata tramite event"**: "Sì" = trigger **dipendente dallo stato** del volo (quota, incidence, massa, apogeo), rilevato con l'opzione `Events` di `ode45` per fermare l'integrazione esattamente al passaggio di zero. "No" = trigger **puramente temporale** (durata assegnata, es. fasi 2 e 5), gestito col limite superiore di `tspan`; anche in queste fasi restano comunque attivi, come rete di sicurezza, gli event globali di terminazione (quota = 0, propellente esaurito), coerentemente con l'overview §1. La fase 3 era erroneamente marcata "No" pur avendo un trigger di stato analogo alla fase 1: corretto in "Sì".

### Tabella unica: fase ↔ `theGuidFlag` ↔ `case guidance.m` ↔ `active_stage`
Sorgente di verità: il **codice** (`guidance.m`). Il flag di output `RES.theGuidFlag` è allineato ai `case`.

| Fase | Nome | `guidance.m` case | `RES.theGuidFlag` | `active_stage` |
|------|------|-------------------|-------------------|----------------|
| 0 | Lift-off | — (non simulata) | — | 1 |
| 1 | Vertical-rise | `case 1` | 1 | 1 |
| 2 | Pitch over | `case 2` | 2 | 1 |
| 3 | Transizione al gravity turn | `case 3` | 3 | 1 |
| 4 | Gravity turn | `case {4,5}` | 4 | 1 (brucia `MOT(1)` fino a esaurimento = trigger) |
| 5 | Coasting | `case {4,5}` | 5 | **2** (switch a fine fase 4: separazione 1° stadio) |
| 6 | Insertion in transfer orbit | `case 6` | 6 | 2 |

### Altri trigger
- Se in una fase **da 1 a 4** il propellente del primo stadio finisce → passa **subito alla fase 5**.
- Se in **qualunque fase** la quota è uguale a zero → **termina** la simulazione.
- Se in **fase 6** finisce il propellente → **termina** la simulazione.

---

## 6. Sistemi di riferimento

Il codice può usare tutti i sistemi di riferimento riconosciuti dalla letteratura tecnica.
Alcuni sono già definiti custom come segue:

### Inerziale Geocentrico — sigla `In`
- **Origine:** centro della Terra
- **Asse-X:** verso il punto di latitudine 0° e longitudine 0° a tempo 0 s
- **Asse-Y:** conseguente per creare una terna destrorsa
- **Asse-Z:** verso il Polo Nord
- **Note:** ECEF *frozen* a tempo 0

### Orizzonte Locale Iniziale — sigla `Ol`
- **Origine:** alle coordinate di lancio
- **Asse-X:** verso EST a tempo 0 s
- **Asse-Y:** verso NORD a tempo 0 s
- **Asse-Z:** lungo la verticale locale, verso uscente
- **Note:** ENU *frozen* a tempo 0

### VNC — sigla `Vn`
- **Origine:** nella posizione del veicolo
- **Asse-X:** verso il vettore velocità
- **Asse-Y:** conseguente per creare una terna destrorsa
- **Asse-Z:** lungo il vettore momento angolare

---

## 7. Interfacce del codice

Le interfacce sono da derivare dal file **`interface_specification.md`**.
Possono essere **adattate** (non rigide).

```
interface(LV, ENV, GUID, TARG, CONSTR)                                -> other   % mapping docx -> codice
eom(t, y, other)                                                      -> dy
simulator(config)                                                     -> RES
guidance(MIS, ENV, GUI, t, pos, vel, AoA, relative_speed, phase)      -> uIn
create_output(T, Y, other)                                            -> RES
plotter(T, Y, RES)                                                    -> <!-- PLACEHOLDER -->
```

**Regola di mapping**: il codice di calcolo (`eom.m`, `guidance.m`) usa **solo** i nomi interni
(`ENV / AER / MOT / GUI / MIS`). La traduzione dai nomi del docx (`LV / GUID / TARG / CONSTR`)
avviene **esclusivamente** in `interface.m`. Non usare i nomi del docx dentro `eom.m`/`guidance.m`.

---

## 8. Glossario variabili canoniche

### Vettore di stato `y` (8 componenti, integrato dall'ODE solver)
| Componente | Nome | Unità | Significato |
|------------|------|-------|-------------|
| `y(1:3)` | `pos` | m | Posizione (frame ECEF/ECI) |
| `y(4:6)` | `vel` | m/s | Velocità |
| `y(7)` | `mass` | kg | Massa istantanea del veicolo |
| `y(8)` | `dv` | m/s | Delta-v accumulato (integrale del modulo dell'accelerazione non gravitazionale) |

### Struct `other` (contenitore passato a `eom.m`)
| Nome | Significato |
|------|-------------|
| `other.ENV` | Struct ambiente (gravità, atmosfera, geodesia) |
| `other.AER` | Struct dati aerodinamici |
| `other.MOT` | Struct dati motore/propulsione (array a 2 elementi) |
| `other.GUI` | Struct parametri di guida |
| `other.MIS` | Struct parametri di missione |
| `other.isignite` | Flag booleano: motore acceso (`true`) o spento (`false`) |
| `other.phase` | Fase di volo corrente (usata dalla guida) |

### `ENV` — ambiente
| Nome | Significato |
|------|-------------|
| `ENV.mu` | Parametro gravitazionale terrestre μ [m³/s²] |
| `ENV.wgs84` | Parametri dell'ellissoide WGS84 (per `cart2geo`) |
| `ENV.altitude` | Vettore quote di riferimento per le tabelle atmosferiche |
| `ENV.atmospheric_density` | Densità atmosferica tabellata vs quota [kg/m³] |
| `ENV.sound_speed` | Velocità del suono tabellata vs quota [m/s] |
| `ENV.ambient_pressure` | Pressione ambiente tabellata vs quota [Pa] |

### `AER` — aerodinamica
| Nome | Significato |
|------|-------------|
| `AER.Mach` | Griglia Mach per l'interpolazione di `Cd` |
| `AER.AoA` | Griglia angolo d'attacco per l'interpolazione di `Cd` |
| `AER.Cd` | Coefficiente di resistenza tabellato `Cd(Mach, AoA)` |
| `AER.Sref` | Superficie aerodinamica di riferimento [m²] |

### `MOT` — motore / propulsione (array a 2 elementi: `MOT(1)`=stadio 1, `MOT(2)`=stadio 2)
Il fasatore seleziona `MOT(stage)` in base a `GUI.active_stage`.

| Nome | Significato |
|------|-------------|
| `MOT(k).mass_flow_rate` | Portata massica di un singolo motore, stadio `k` [kg/s] |
| `MOT(k).vacuum_thrust` | Spinta nel vuoto di un singolo motore, stadio `k` [N] |
| `MOT(k).nozzle_exit_area` | Area di uscita dell'ugello, stadio `k` [m²] |
| `MOT(k).number_of_ignite_engine` | Numero di motori accesi, stadio `k` |

### `GUI` — parametri di guida
| Nome | Significato |
|------|-------------|
| `GUI.zkick` | Quota di fine vertical-rise (trigger fase 1) |
| `GUI.InOl` | Matrice di rotazione dal frame `Ol` al frame `In` |
| `GUI.last_pitch` | Ultimo valore di pitch di comando [rad] |
| `GUI.last_yaw` | Ultimo valore di yaw di comando [rad] |
| `GUI.launch_azimuth` | Azimut di lancio imposto (usato come yaw) [rad] |
| `GUI.active_stage` | Stadio attivo corrente (`1` o `2`); seleziona `MOT(stage)` in `eom.m`. Inizializzato a 1 in `interface.m`, portato da 1 a 2 da `simulator.m` a fine fase 4 (separazione 1° stadio per esaurimento propellente) |
| `GUI.pitch_over_starting` | Istante di inizio della fase di pitch over [s] |
| `GUI.pitch` | Coefficienti del profilo di pitch: `pitch(1)`·dt² + `pitch(2)`·dt |
| `GUI.transition_starting` | Istante di inizio della transizione al gravity turn [s] |
| `GUI.pitch_rate_transition` | Rate di pitch durante la transizione [rad/s] |
| `GUI.pitch_at_transition` | Pitch iniziale all'inizio della transizione [rad] |
| `GUI.insertion_starting` | Istante di inizio della fase di insertion [s] |
| `GUI.AoA_rate` | Rate dell'angolo d'attacco comandato in insertion [rad/s] |
| `GUI.plane_controller` | Guadagni del controllore di piano [kp, kd, ki] |

### `MIS` — parametri di missione
| Nome | Significato |
|------|-------------|
| `MIS.apogee_altitude_target` | Quota di apogeo obiettivo (trigger fase 6) [m] |
| `MIS.target_orbital_inclination` | Inclinazione orbitale obiettivo [rad] |

---

## 9. Definition of Done (checklist di validazione)

Prima di dichiarare completato un task, Claude deve verificare che:

- [ ] Il codice sia **compatibile Octave + Matlab 2025A** (nessun `endif`/`endfor`/`#`).
- [ ] Non ci siano **variabili globali**.
- [ ] Gli output non contengano **`NaN` né numeri immaginari** non gestiti.
- [ ] Le terne di riferimento siano **destrorse** dove richiesto.
- [ ] I file I/O siano **CSV delimitati da spazio** nelle cartelle corrette.
- [ ] La coerenza fase ↔ `theGuidFlag` ↔ `active_stage` rispetti la tabella unica (§5).
- [ ] Il caso di test definito da Claude (§10) giri end-to-end senza errori e rispetti i criteri di validazione.

---

## 10. Definizione del caso di test (a cura di Claude)

Il repository **non fornisce** un caso di test pronto: **Claude deve definirlo autonomamente**.
Quando serve validare il codice, Claude costruisce un caso di test seguendo queste regole.

### 10.1 Cosa deve produrre Claude
- Un **dataset di input completo** nella cartella `/input/validation_test`, in formato **CSV delimitato da spazio**, coerente con tutte le struct di input della specifica (`LV / ENV / GUID / TARG / CONSTR`).
- I valori devono essere **fisicamente plausibili** per un lanciatore a due stadi a liquido (ordini di grandezza realistici per masse, spinte, portate, quote, azimut).
- Dove mancano dati reali, Claude **sceglie valori ragionevoli e li documenta** con un commento/nota accanto al test, senza inventare campi non previsti dalla specifica.

### 10.2 Criteri che il test deve verificare
Il caso di test deve permettere di controllare almeno:
- **Attraversamento completo delle fasi**: la simulazione passa da fase 1 a fase 6 senza bloccarsi (la fase 0 non è simulata, vedi §5).
- **Trigger corretti**: ogni passaggio di fase avviene per il trigger previsto (§5), non per un evento spurio.
- **Staging**: `active_stage` passa da 1 a 2 a fine fase 4 (separazione 1° stadio per esaurimento propellente), e `eom.m` usa `MOT(2)` dalla fase 5 in avanti.
- **Sanità numerica**: nessun `NaN` né numero immaginario negli stati e negli output.
- **Coerenza fisica**: massa monotòna non crescente; quota ≥ 0; delta-v crescente; terne di riferimento destrorse.
- **Chiusura di missione**: raggiungimento di `MIS.apogee_altitude_target` (o terminazione corretta per esaurimento propellente / quota nulla).
- **Round-trip del mapping**: `interface.m` produce un `other` completo (tutti i campi usati da `eom.m`/`guidance.m` sono popolati).

### 10.3 Come documentare il test
- Claude deve indicare **cosa verifica** ogni file/valore del caso di test.
- Deve riportare gli **output attesi** (o range attesi) per i controlli sopra, così che il test sia ripetibile.
- Se un criterio non è verificabile con i dati scelti, deve **dichiararlo esplicitamente** anziché assumerlo superato.
