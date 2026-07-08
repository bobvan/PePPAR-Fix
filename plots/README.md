# plots/

Output directory for the `tools/plot_*.py` visualizers. The generated
`*.html` files are **not committed** — they are Plotly exports that inline
the full plotly.js bundle (~4.9 MB each) and are regenerable from capture
data via the project's own tooling. `.gitignore` excludes `plots/*.html`;
this README keeps the directory present so the tools (which write to
hardcoded `plots/...` paths) work on a fresh checkout.

Regenerate a plot by running its tool, e.g.:

```sh
pip install plotly            # not in the default deps; needed by these tools
python tools/plot_deviation.py ...
python tools/plot_freerun_characterization.py -o plots/freerun-char-<date>
```

Source captures live on gt (`/home/bob/gt/`), not in this repo. Any
previously committed plot HTML remains recoverable from git history:

```sh
git log --oneline -- plots/<name>.html
git show <commit>:plots/<name>.html > plots/<name>.html
```
