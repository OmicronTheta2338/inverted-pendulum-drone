# Design case-study workspace

This directory is the working home for a portfolio-style account of the inverted-pendulum
drone. The intended result is an engineering design case study, not a journal paper: it should
be readable in one sitting, visually led, technically defensible, and candid that the current
validation is in simulation while the physical airframe is still being built.

## Working title

**Keeping the Base Level: Designing a Drone to Swing Up and Balance a Double Inverted
Pendulum**

The central question is:

> How can a flying base balance a double inverted pendulum without tilting underneath it, and
> how much sensing and actuator latency can the design tolerate?

That question gives the write-up a better spine than a chronological project diary. The design
history still matters, but each decision should appear when it helps answer the question.

## What's here

- [`document-plan.md`](document-plan.md) — audience, story, section-by-section outline, and
  staged publication plan.
- [`evidence-plan.md`](evidence-plan.md) — prioritized figure, graph, illustration, and video
  register.
- [`capture-guide.md`](capture-guide.md) — reproducible simulation captures now, followed by
  bench and flight measurements later.
- [`claim-register.md`](claim-register.md) — what can be said now, what evidence supports it,
  and what must remain provisional.
- [`draft/case-study.md`](draft/case-study.md) — a writing scaffold with a drafted opening and
  figure slots.
- [`data/`](data/) — immutable raw data, derived data, run metadata, and the schema for future
  full-state logs.
- [`figures/`](figures/) — editable figure sources and publication-ready exports.
- [`recordings/`](recordings/) — raw captures and edited video.
- [`notes/`](notes/) — interview notes, build diary entries, captions, and discarded ideas.

The repository root remains the source for controller and simulator design notes. Material in
this directory turns that record into a coherent public story; it should not silently replace
the technical source documents.

## Evidence labels

Use one of these labels anywhere a result might otherwise be ambiguous:

- **Analysis** — derived from equations, geometry, or a component specification.
- **Simulation** — measured in the Godot/Jolt or idealized plant.
- **Bench** — measured on restrained hardware or a component test fixture.
- **Flight** — measured on the free-flying vehicle.
- **Provisional** — an assumption or estimate awaiting measurement.

In the finished case study, labels can be subtle caption prefixes rather than badges everywhere.
They exist to prevent a simulation result from quietly turning into a hardware claim.

## Asset rules

1. Keep raw data and original recordings unchanged after capture.
2. Record the commit, exact command, environment overrides, seed, and result beside every run.
3. Generate final plots from files in `data/raw/` or `data/processed/`, never from numbers copied
   out of prose.
4. Put units in column names or metadata and on every plotted axis.
5. Prefer SVG or PDF for diagrams and plots; use PNG for raster screenshots and MP4 for video.
6. A failed run is evidence. Archive representative failures when they explain a design change.
7. Use the claim register before tightening wording in the draft.

## Recommended first deliverable

Publish a **simulation edition** once the priority-1 evidence in `evidence-plan.md` exists. It
can end with the detailed CAD and hardware validation plan. Update the same case study with
bench and flight evidence later rather than waiting for the entire vehicle to be finished.

## Best next working session

1. Resolve the dated configuration and wording discrepancies in `claim-register.md`.
2. Add the optional integrated full-state CSV described in `capture-guide.md`.
3. Capture V05–V09 and populate `data/experiment-manifest.csv`.
4. Draw V01–V04 in one consistent visual language.
5. Replace the TODO tables and figure comments in `draft/case-study.md` with evidence links.
