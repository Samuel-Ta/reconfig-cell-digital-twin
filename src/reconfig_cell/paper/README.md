# Paper — surrogate-validated reconfigurable cell

`main.tex` is an IEEE conference paper (`IEEEtran`, `conference` option), authors omitted.

## Build
This machine has no `pdflatex`/`IEEEtran.cls`. Either:
- **Overleaf** (recommended): upload `main.tex` + `figs/` (IEEEtran is built in), or
- locally: `sudo apt install texlive-latex-recommended texlive-publishers texlive-science`
  then `pdflatex main && pdflatex main` (twice, for refs).

## Figures (`figs/`)
| file | source |
|---|---|
| `gate_scatter.png`   | `val_hb/figs/surrogate_vs_real.png` — gate scatter, r=0.880 |
| `sa_convergence.png` | `opt_out_gate/figs/sa_convergence.png` — SA cooling |
| `sa_vs_baseline.png` | `opt_out_gate/figs/sa_vs_baseline.png` — SA vs baseline 5/5 |
| `closed_loop.png`    | `val_loop/figs/closed_loop.png` — pooled correlation n=11 |

## Data provenance (all numbers traceable)
- Table I/II (gate): `val_hb/motion.csv`, `cell_synth/SURROGATE_VS_REAL.md` Attempt 3.
- Table III (SA): `opt_out_gate/opt_summary_n3.csv`.
- Table IV (closed loop): `val_loop/motion.csv` + `val_hb/motion.csv`.
- Table V (twin): `/tmp/demo_val0.csv` (warmup 133.0s, trial 148.0s, both success).
