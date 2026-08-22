# Domain Docs

Before exploring, read the root `CONTEXT.md`, or relevant contexts from
`CONTEXT-MAP.md`, plus applicable ADRs under `docs/adr/`.

Missing domain files are not errors. Proceed silently; domain-modeling skills
create them lazily when terminology or architectural decisions are resolved.

This repository uses the single-context layout:

```text
/
|-- CONTEXT.md
|-- docs/adr/
`-- src/
```

Use terminology defined in `CONTEXT.md`. Explicitly flag proposals that
contradict an existing ADR rather than silently overriding it.
