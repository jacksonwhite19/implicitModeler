# Small robustness proposal — NOT RUN

Four fixed cases, three existing Rhai geometries. No source query, candidate construction or mesh run was performed to prepare this proposal. Run only under separate execution authorization after consolidation closure.

| Order | Geometry | Scale (m/model unit) | Nominal surface (m) | Question |
|---|---|---:|---:|---|
| R1 | ellipsoid | 0.001 | 0.0002 | Anisotropic curvature; new V3.5 coverage. |
| R2 | concave | 0.001 | 0.0002 | Concavity and nearby fronts; historical V4 pass, new V3.5 coverage. |
| R3 | concave | 0.00075 | 0.0002 | Smaller geometry at fixed physical prism and surface settings; relative-clearance stress, not unit invariance. |
| R4 | passage | 0.001 | 0.0001 | Finer nominal surface size; new resolution coverage beyond accepted nodal 200 um passage. |

All use explicit nodal mode, first height 20 um, nominal growth 1.3, five rows, fixed position tolerance 50 um and normal tolerance 15 degrees. Other surface/seed/projection and direct-background controls retain the frozen template. R3 keeps physical settings fixed while shrinking geometry; R4 changes nominal surface resolution only. These settings are hypotheses, not qualified recommendations.

The copied complete_cli210 folder was prepared only. Separate phaseB233 execution passed the historical V4 engineering contract; its frozen sdf hash matches these Rhai bytes. That historical pass is not current V3.5 qualification.

First scientific, identity or resource refusal stops the sequence. No retry, tuning, source change or scope expansion. Resource failure is operationally inconclusive. Unchanged full fresh pipeline and current gates, independent saved-artifact readback and clean source/resource closure. Report achieved layers/compression and all warnings. 43200 seconds, 16 GiB sampled Windows process-tree RSS; Linux limit is per-process RLIMIT_AS; 16000000000 output bytes per case.

Run from the standalone directory with the sibling qualification folder preserved:

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry ../qualification/consolidation_20261027/robustness_proposal/inputs/ellipsoid.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-position-error 0.00005 --surface-normal-angle-deg 15 --surface-carrier nodal --background fixtures/background_direct_v4.json --output results/robustness_proposal_r1_01 --stop-after complete --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry ../qualification/consolidation_20261027/robustness_proposal/inputs/concave.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-position-error 0.00005 --surface-normal-angle-deg 15 --surface-carrier nodal --background fixtures/background_direct_v4.json --output results/robustness_proposal_r2_01 --stop-after complete --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry ../qualification/consolidation_20261027/robustness_proposal/inputs/concave.rhai --metres-per-model-unit 0.00075 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-position-error 0.00005 --surface-normal-angle-deg 15 --surface-carrier nodal --background fixtures/background_direct_v4.json --output results/robustness_proposal_r3_01 --stop-after complete --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry ../qualification/consolidation_20261027/robustness_proposal/inputs/passage.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0001 --surface-policy size-led --surface-position-error 0.00005 --surface-normal-angle-deg 15 --surface-carrier nodal --background fixtures/background_direct_v4.json --output results/robustness_proposal_r4_01 --stop-after complete --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```
