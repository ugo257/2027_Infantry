# Issue tracker: GitHub

Issues and specs for this repo live as GitHub issues. Use the `gh` CLI for all operations.

## Conventions

- Create: `gh issue create --title "..." --body "..."`
- Read: `gh issue view <number> --comments`
- List: `gh issue list --state open --json number,title,body,labels,comments`
- Comment: `gh issue comment <number> --body "..."`
- Label: `gh issue edit <number> --add-label "..."` or `--remove-label "..."`
- Close: `gh issue close <number> --comment "..."`
- Infer the repository from the current clone and `git remote -v`.

## Pull requests as a triage surface

**PRs as a request surface: no.**

A bare `#42` may refer to an issue or PR. Try `gh pr view 42`, then fall back to
`gh issue view 42`.

## Skill terminology

- "Publish to the issue tracker": create a GitHub issue.
- "Fetch the relevant ticket": run `gh issue view <number> --comments`.

## Wayfinding

Use one `wayfinder:map` issue with linked child issues. Child labels use
`wayfinder:research`, `wayfinder:prototype`, `wayfinder:grilling`, or
`wayfinder:task`. Prefer native GitHub sub-issues and issue dependencies; fall
back to task lists and `Blocked by: #<n>` when unavailable. Claim work by
assigning the issue to `@me`.
