# BWB EDF Vehicle Prompt

Design a sleek blended-wing-body EDF aircraft with these constraints and proportions:

- Configuration: blended flying wing / lifting body with a very smooth body-wing transition
- Mission: internal-EDF loitering platform
- Wingspan: 800 mm
- Overall length: 500 mm
- Maximum body width: 172 mm
- Maximum body height: 95 mm
- General shape: balanced BWB, narrow and wide centerbody, no separate tube fuselage
- Wing planform: one continuous swept wing, not cranked, not double-diamond
- Wing sweep: 39 degrees
- Wing root chord: 310 mm
- Wing tip chord: 62 mm
- Wing section: use a reasonable tailless-friendly section
- Visual intent: sleek, clean, integrated UAV

Tail:

- Tail type: practical V-tail
- Tail placement: aft corners of the body
- Tail size: about 30% larger than the previous medium baseline
- V-tail span: about 179 mm
- V-tail root chord: about 107 mm
- V-tail tip chord: about 55 mm
- V-tail sweep: moderate

Inlet and ducting:

- Dorsal inlet
- Inlet style: conformal rounded rectangle, raised above the upper mold line
- Inlet outer mouth sized proportionally to body width
- Inlet inner mouth follows the same rounded-rectangle family, not an ellipse
- Inlet face should be clean and open, with no sealing cap or artifact
- Lower lip of inlet should sit above the fuselage crown
- Duct should begin at the dorsal inlet and descend relatively quickly
- Early duct path should not crest above the inlet face
- Duct should be a real through-body duct, not just a scoop or shell on top
- Internal outlet: circular 70 mm duct
- Do not model the EDF hardware itself

Structure:

- Hollow shell
- Shell thickness: about 1.6 mm baseline, but lightweight
- Include medium-density internal structure
- Add wing ribs
- Add fuselage bulkheads
- Structure should be trimmed correctly around the duct
- Do not model electronics, EDF hardware, payloads, or landing gear
- Do not model control surfaces

Modeling priorities:

- Clean external BWB silhouette
- Strong, believable body-wing blending
- Continuous swept wing
- Proper dorsal inlet and internal duct continuity
- Smooth internal duct path
- Practical printable shell and internal structure
- Avoid weird cranked, diamond, or faceted wing geometry
