# Hardware Labels for Beads (`hw:` convention)

Hardware-dependent beads carry `hw:` labels so the Mayor can schedule work
without resource conflicts. This replaces a lock service — the Mayor checks
labels before slinging, and bead lifecycle (close → free) handles release.

See decision: `pf-zx9` (Lab resource allocation).

## Convention

Labels use the prefix `hw:` followed by the resource's key from
`timelab/resources.json`. A bead may carry multiple labels if it requires
several hardware resources simultaneously.

### Defined labels

| Label | Resource | Type | Notes |
|-------|----------|------|-------|
| `hw:TimeHat` | TimeHat (Pi 5 + TimeHAT v5) | host+NIC | Includes the i226 PHC |
| `hw:F9T-3RD` | u-blox ZED-F9T EVK | receiver | On TimeHat USB |
| `hw:TICC-3` | TAPR TICC #3 | time interval counter | On TimeHat USB |
| `hw:Patch2` | Patch2 antenna | antenna | Roof-mounted, L1/L5 |
| `hw:E810` | Intel E810-XXVDA4T | NIC | On order (2026-03-17) |
| `hw:SFN6322F-1` | Solarflare SFN6322F #1 | NIC | In inventory |
| `hw:SFN6322F-2` | Solarflare SFN6322F #2 | NIC | In inventory |
| `hw:SFN8522-1` | Solarflare SFN8522 #1 | NIC | In inventory, license unknown |
| `hw:SFN8522-2` | Solarflare SFN8522 #2 | NIC | In inventory |
| `hw:SFN8522-3` | Solarflare SFN8522 #3 | NIC | In inventory |
| `hw:SFN8522-4` | Solarflare SFN8522 #4 | NIC | In inventory |

### Adding new labels

When new hardware arrives, add it to `timelab/resources.json` with an
`hw_label` field and update this table. The label must match the JSON key
(e.g., key `"F9T-3RD"` → label `hw:F9T-3RD`).

## Mayor scheduling protocol

Before slinging hardware-dependent work, the Mayor:

1. **Checks current usage**: `bd list --labels=hw:<resource>` — if any open/in_progress
   beads carry the label, the resource is in use.
2. **Sets labels on the new bead**: `bd create --labels hw:TimeHat,hw:F9T-3RD "..."`
3. **Adds dependency if blocked**: if the resource is busy, the new bead gets a
   `blocked-by` dependency on the bead that holds it.
4. **Release is automatic**: when the polecat closes its bead (`bd close`), the
   `hw:` labels free up and `bd ready` shows newly unblocked work.

## Examples

```bash
# A bead that needs the F9T and TimeHat:
bd create --labels hw:TimeHat,hw:F9T-3RD --title "Run 1-hour PPP observation"

# Check if the TICC is free:
bd list --labels=hw:TICC-3
# Empty result → TICC is available

# A bead that needs the E810 (once it arrives):
bd create --labels hw:E810 --title "Qualify E810 PPS I/O with DKMS driver"
```

## Deacon topology patrol

The Deacon periodically validates `timelab/resources.json` against reality
by SSH-ing to hosts and checking device presence (`ls /dev/gnss-top`,
`ls /dev/ticc`, `ls /dev/ptp*`). Discrepancies are filed as beads.
