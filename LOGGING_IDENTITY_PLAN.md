# Board identity and log naming plan

This is a forward-looking design note. It is intentionally not part of the
current flight firmware or today's downloaded data.

## Identity

Each physical board is identified by its stable Wi-Fi/base MAC address. The
runtime should derive a board key from the least-significant four hexadecimal
digits of that MAC and use a board profile table to select instance-specific
calibration and alignment data. The profile lookup key is the full MAC; the
four-digit suffix is only a compact human-facing identifier.

## Log filename

New log files should use `<HAL><MAC4><NNN>.bin`: `G` for GEEK, `T` for T-Beam,
`MAC4` as uppercase least-significant MAC digits, and `NNN` as a zero-padded
per-board sequence. Examples: `G247C001.bin`, `TCCB8007.bin`.

The HAL flavor identifies base hardware and driver selection, not calibration.

## Persistent sequence number

The counter must live outside the SD card so it survives SD swaps, formatting,
and replacement. Preferred storage is an NVS/Preferences record in on-chip
flash; SPIFFS/LittleFS is acceptable if required by project conventions.

Key it by the full board MAC, increment transactionally before creating a log,
and never derive it by scanning the SD card. After `999`, continue with decimal
sequence values; parsers must not assume exactly three digits.

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

- Continue reading current `fusion-*.bin` names and record format.
- Add metadata at log start before switching to new names.
- Preserve an import/rename path for old files; do not infer board identity
  from filenames alone.
- Keep LIST/DUMP filename handling opaque and validated, not tied to a fixed
  `fusion-####.bin` pattern.

## Bookmark

Before implementation, settle the exact MAC source, NVS namespace and atomic
update strategy, board-profile schema, metadata record ABI, and legacy replay
override. The current T-Beam flight batch is intentionally unchanged.
