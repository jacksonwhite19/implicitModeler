# Dashboard

Purpose: standalone local web application for monitoring and reviewing optimization campaigns.

Expected views:

- Campaign list.
- Run monitor.
- Candidate browser.
- Candidate detail page.
- Ranking.
- Convergence plots.
- Pareto plots.
- Artifact gallery.
- Failure analysis.
- Lineage view.
- Human annotations.

Constraint:

- Dashboard reads from the run database and artifact store. It must not become the record of truth.

## Mobile Read-Only Monitor

`mobile_monitor.py` is a small stdlib-only web monitor for checking a running
optimizer campaign from a phone. It does not control the optimizer and does not
write to run records.

It shows export, mesh/potential, no-slip steady solver, alpha case, ranking,
and failure counts as artifacts appear. For the current no-slip rough-scoring
backend, each requested alpha creates a laminar starter case and an SST-started
case. A `14` candidate backend run with `0/4/10 deg` first-pass alphas can
therefore create up to `84` alpha case directories before final ranking appears.

Current 15-candidate run:

```powershell
python .\aircraft_optimizer_platform\software\dashboard\mobile_monitor.py `
  --run-name real_optimizer_15candidate_full_20260628 `
  --host 0.0.0.0 `
  --port 8765
```

Open `http://<this-computer-ip>:8765/` from a device that can reach this
machine. If ngrok is running, open the active ngrok forwarding URL instead.
For the 2026-06-28 15-candidate run, the active tunnel is:

```text
https://tetched-charlie-dextrously.ngrok-free.dev
```

Ngrok may show a browser warning page first; click through once. The dashboard
uses the `ngrok-skip-browser-warning` header for its JSON refresh requests.

The JSON endpoint is:

```text
/api/status
```

Status: bare-bones mobile monitor implemented; full dashboard remains future
work.
