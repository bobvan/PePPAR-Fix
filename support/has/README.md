# Galileo HAS decoding support data

Reference data from the **Galileo HAS Signal-in-Space ICD v1.0** (EUSPA,
May 2022), redistributed via the open `hirokawa/cssrlib-data` repo.

- `has_gmat.csv` — Annex B Reed-Solomon generator matrix (255×32, GF(256)).
  Load-bearing: `tools/has_decode_cssr.py` / CSSRlib needs it to RS-decode
  HAS pages.
- `has_annexD_example.txt` — Annex D worked decoding example; a self-test
  vector for the decoder toolchain.

Pipeline: `tools/has_page_monitor.py --raw-pages` (capture E6 pages off the
X20) → `tools/has_decode_cssr.py` (RS + SSR decode via CSSRlib).  See
`docs/galileo-has-research.md`.
