# Board identity and log naming

The filename and persistent sequence portions of this design are implemented.
The richer board-profile and log-metadata portions remain future work.

## Identity

Each physical board is identified by its stable Wi-Fi/base MAC address. The
runtime should derive a board key from the least-significant four hexadecimal
digits of that MAC and use a board profile table to select instance-specific
calibration and alignment data. The profile lookup key is the full MAC; the
four-digit suffix is only a compact human-facing identifier.

## Log filename

New log files use `<HAL><MAC4><NNN>.bin`: `G` for GEEK,
`MAC4` as uppercase least-significant MAC digits, and `NNN` as a zero-padded
per-board sequence. Example: `G247C001.bin`.

The HAL flavor identifies base hardware and driver selection, not calibration.

## Persistent sequence number

The counter lives in an NVS/Preferences record in on-chip flash so it survives
SD swaps, formatting, replacement, and firmware updates.

It is keyed by the full board MAC and reserved before creating a log. A failed
creation can leave a harmless gap, but a number is never reused. After `999`,
the decimal sequence grows naturally; parsers do not assume exactly three
digits.

## Calibration ownership

1. HAL: board family, pins, buses, sensor drivers, and base conventions.
2. Board profile: MAC-specific axis remaps, mounting rotation, offsets,
   compass matrices, and calibration version.
3. Log metadata: full MAC/profile identifier, HAL flavor, sensor identities,
   calibration revision, and log-format version.

Replay should select the board profile from recorded metadata, with explicit
command-line overrides for legacy or forensic work. Logs without metadata must
use an explicit legacy-profile option rather than silent guessing.

## Migration and compatibility

- Continue reading legacy `fusion-*.bin` names and the existing record format.
- Add board/profile metadata to the log start event in a future format update.
- Preserve an import/rename path for old files; do not infer board identity
  from filenames alone.
- Keep LIST/DUMP filename handling opaque and validated, not tied to a fixed
  `fusion-####.bin` pattern.

## Remaining work

The board-profile schema, metadata record ABI, calibration revision, and
legacy replay override are not yet implemented.
