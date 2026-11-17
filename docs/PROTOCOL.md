# RoboTwin-Protokoll

Version **1** (mit Erweiterung Not-Halt aus M7 – abwärtskompatibel: neue Befehle und Felder,
die ältere Clients ignorieren)

Dieses Dokument beschreibt, wie ein Client (Unity, Test-Client) mit dem RoboTwin-Controller
spricht. Es ist der Vertrag zwischen beiden Seiten: Was hier steht, gilt – unabhängig davon,
in welcher Sprache ein Client geschrieben ist.

## 1. Transport

| Eigenschaft | Wert |
|---|---|
| Verbindung | TCP |
| Standard-Adresse | `127.0.0.1:5005` (nur lokal erreichbar; für Zugriff aus dem Netz `--host 0.0.0.0`) |
| Zeichenkodierung | UTF-8 |
| Rahmung | **eine JSON-Nachricht pro Zeile**, abgeschlossen mit `\n` |
| Maximale Zeilenlänge | 64 KiB |

TCP liefert einen Bytestrom ohne Nachrichtengrenzen. Die Zeilenenden sind die Grenzen:
Ein Empfänger liest bis `\n` und erhält genau eine Nachricht. Nachrichten werden daher
kompakt (ohne Zeilenumbrüche) geschrieben.

Jede Nachricht ist ein JSON-Objekt mit dem Feld `"type"`. **Unbekannte Felder werden
ignoriert** – so können spätere Versionen Felder ergänzen, ohne ältere Clients zu brechen.

## 2. Einheiten und Konventionen

| Größe | Einheit / Format |
|---|---|
| Gelenkwinkel | **Grad**, Vorzeichen nach URDF (rechtshändig) |
| Gelenkgeschwindigkeit | Grad/s |
| Gelenkbeschleunigung | Grad/s² |
| `speed` | Anteil der Maximalgeschwindigkeit, `0 < speed ≤ 1` |
| Override | Prozent, `1 … 100` |
| `timestamp` | Sekunden seit Start des Controllers (monotone Uhr) |
| Reihenfolge der Gelenke | Basis → Handgelenk 3 (`shoulder_pan` … `wrist_3`), siehe `hello` |

- Zahlen sind JSON-Zahlen: **Punkt** als Dezimaltrennzeichen, unabhängig von der Sprache des Systems.
- `NaN` und `Infinity` sind in JSON nicht erlaubt und werden nie gesendet.
- `id`: ganze Zahl ≥ 1, vom Client vergeben. Der Controller übernimmt sie unverändert in die Antwort.

## 3. Ablauf

```
Client                                   Controller
  │ ─────────── TCP-Verbindung ──────────▶ │
  │ ◀── hello (Version, Rolle, Grenzen) ── │   sofort nach dem Verbinden
  │ ◀──────────── state ────────────────── │   50 Hz, an alle Clients
  │ ──── move_joints (id 1) ─────────────▶ │
  │ ◀──────────── ack (id 1) ───────────── │   Befehl angenommen (nicht: fertig!)
  │ ◀──────────── state (moving) ───────── │
  │ ◀──────────── state (idle) ─────────── │   Bewegung beendet
```

`ack` bedeutet **„Befehl angenommen“**, nicht „Bewegung abgeschlossen“. Das Ende einer Bewegung
erkennt der Client am Status `idle` in den `state`-Nachrichten.

## 4. Rollen: Besitzer und Beobachter

Mehrere Clients dürfen gleichzeitig verbunden sein. Alle erhalten `state`.

- Der **erste** Client wird **Besitzer** (`owner`) und darf steuern.
- Alle weiteren sind **Beobachter** (`observer`). Ihre Befehle werden mit `not_owner` abgelehnt.
- **Ausnahme: `stop` und `emergency_stop` darf jeder senden.** Anhalten muss von überall möglich sein.
- `reset` (Not-Halt quittieren) darf nur der Besitzer – oder jemand an der Konsole des Controllers.
- Trennt sich der Besitzer, **hält der Roboter an**, und der am längsten verbundene Beobachter
  wird neuer Besitzer. Er erhält die Nachricht `role`.

## 5. Nachrichten vom Client zum Controller

### `move_joints` – alle Gelenke synchron zu Zielwinkeln fahren

```json
{"type":"move_joints","id":1,"joints":[0,-90,90,-90,-90,0],"speed":1.0}
```

| Feld | Typ | Pflicht | Bedeutung |
|---|---|---|---|
| `id` | int | ja | Befehls-Id |
| `joints` | Zahl[6] | ja | Zielwinkel in Grad |
| `speed` | Zahl | nein (Standard 1.0) | Anteil der Maximalgeschwindigkeit |

Eine laufende Bewegung wird ersetzt: Der Controller plant vom aktuellen Zustand aus neu.
Alle Gelenke starten und erreichen das Ziel gleichzeitig (synchrones Trapezprofil).
Liegt ein Zielwinkel außerhalb der Achsgrenzen, wird der **ganze Befehl abgelehnt**
(`joint_limit`) – der Roboter fährt dann nicht, statt still zu begrenzen.

### `stop` – sofort anhalten

```json
{"type":"stop","id":2}
```

Darf von jedem Client gesendet werden (auch von Beobachtern).

### `set_override` – Geschwindigkeits-Override

```json
{"type":"set_override","id":3,"percent":50}
```

| Feld | Typ | Pflicht | Bedeutung |
|---|---|---|---|
| `id` | int | ja | Befehls-Id |
| `percent` | Zahl | ja | 1 … 100 |

Wirkt auch auf eine laufende Bewegung. Der Override begrenzt nur die Geschwindigkeit,
nicht die Beschleunigung.

### `emergency_stop` – Not-Halt

```json
{"type":"emergency_stop","id":4}
```

Darf von jedem Client gesendet werden. Der Roboter hält **sofort** an (Stopp-Kategorie 0) und
geht in den Fehlerzustand: `state.status` ist `error`, `state.error` nennt den Grund.
Bis zum Quittieren wird jeder `move_joints` mit `emergency_stop_active` abgelehnt;
`stop` und `set_override` bleiben erlaubt.

Auch im Konsolenfenster des Controllers kann der Not-Halt ausgelöst werden (`estop`) –
wie der Not-Halt-Taster am Schaltschrank, unabhängig von jedem Client.

### `reset` – Not-Halt quittieren

```json
{"type":"reset","id":5}
```

Nur der Besitzer (oder `reset` in der Konsole des Controllers). Hebt den Fehlerzustand auf;
ohne aktiven Not-Halt passiert nichts (wird trotzdem mit `ack` bestätigt).

### `move_linear` – reserviert

Lineare Bewegung des TCP. In Version 1 **nicht umgesetzt**; der Controller antwortet mit
`not_implemented`.

## 6. Nachrichten vom Controller zum Client

### `hello` – nach dem Verbinden

```json
{"type":"hello","protocol":1,"controller":"RoboTwin Python Controller","role":"owner",
 "state_rate":50,
 "joints":[{"name":"shoulder_pan_joint","min":-360,"max":360,"max_speed":180,"max_acceleration":180}, …]}
```

(Hier zur Lesbarkeit umbrochen – gesendet wird eine einzige Zeile.)

| Feld | Bedeutung |
|---|---|
| `protocol` | Protokollversion. Passt sie nicht, sollte der Client die Verbindung trennen. |
| `role` | `owner` oder `observer` |
| `state_rate` | Takt der `state`-Nachrichten in Hz |
| `joints` | Name und Grenzen jedes Gelenks, in Protokollreihenfolge |

### `role` – Rolle hat sich geändert

```json
{"type":"role","role":"owner"}
```

### `ack` – Befehl angenommen

```json
{"type":"ack","id":1}
```

### `error` – Befehl abgelehnt oder Nachricht ungültig

```json
{"type":"error","id":1,"code":"joint_limit","message":"Gelenk 3: 200.0° außerhalb [-180°, 180°]."}
```

`id` ist `null`, wenn die Nachricht so kaputt war, dass keine Id gelesen werden konnte.
`code` ist für Programme gedacht, `message` für Menschen.

### `state` – Ist-Zustand, zyklisch an alle Clients

```json
{"type":"state","seq":1234,"timestamp":24.68,"status":"moving","override":50,
 "joints":[12.5,-80.1,95.0,-90.0,-90.0,3.2],"error":null}
```

| Feld | Bedeutung |
|---|---|
| `seq` | fortlaufende Nummer der `state`-Nachricht |
| `timestamp` | Zeit des Controllers in Sekunden |
| `status` | `idle`, `moving` oder `error` (Not-Halt aktiv) |
| `override` | aktueller Override in Prozent |
| `joints` | Ist-Winkel in Grad |
| `error` | Grund des Fehlerzustands (z. B. `"Not-Halt von 127.0.0.1:50000"`), sonst `null` |

## 7. Fehlercodes

| Code | Bedeutung |
|---|---|
| `invalid_json` | Zeile ist kein gültiges JSON-Objekt |
| `invalid_command` | Pflichtfeld fehlt, falscher Typ oder Wert außerhalb des erlaubten Bereichs |
| `unknown_command` | unbekannter `type` |
| `not_implemented` | Befehl ist reserviert, aber noch nicht umgesetzt |
| `not_owner` | Befehl von einem Beobachter (außer `stop` und `emergency_stop`) |
| `emergency_stop_active` | Fahrbefehl während eines aktiven Not-Halts |
| `joint_limit` | Zielwinkel außerhalb der Achsgrenzen |

Clientseitig verwendet Unity zusätzlich `not_connected`, wenn ohne Verbindung gesendet wird.
Dieser Code wird nie vom Controller verschickt.

## 8. Beispiel einer Sitzung

```
→ = Client an Controller, ← = Controller an Client

← {"type":"hello","protocol":1,"controller":"RoboTwin Python Controller","role":"owner","state_rate":50,"joints":[…]}
← {"type":"state","seq":1,"timestamp":0.02,"status":"idle","override":100,"joints":[0,0,0,0,0,0],"error":null}
→ {"type":"move_joints","id":1,"joints":[0,-90,90,-90,-90,0]}
← {"type":"ack","id":1}
← {"type":"state","seq":2,"timestamp":0.04,"status":"moving","override":100,"joints":[0,-0.07,0.07,-0.07,-0.07,0],"error":null}
  …
→ {"type":"move_joints","id":2,"joints":[0,0,200,0,0,0]}
← {"type":"error","id":2,"code":"joint_limit","message":"Gelenk 3: 200.0° außerhalb [-180°, 180°]."}
→ {"type":"stop","id":3}
← {"type":"ack","id":3}
← {"type":"state","seq":75,"timestamp":1.50,"status":"idle","override":100,"joints":[0,-58.1,58.1,-58.1,-58.1,0],"error":null}
```
