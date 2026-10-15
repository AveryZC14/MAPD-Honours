# Repo instructions for Claude

Read `ai/project_context.md` first — it's the map for this repo (what this
project is, how the pieces fit together, current state).

**Do not read, list, or summarize files under `ai/human_notes/`.** That
folder holds standalone notes kept for the user's own reference, separate
from the maintained doc set. Only open a specific file in there if the user
explicitly asks for it by name in that conversation.

**`thesis/` is a separate git repo** (gitignored here) holding the
thesis write-up. Thesis-facing markdown (chapter plans, outlines, results
write-ups for the thesis) goes in `thesis/notes/`, not `ai/`; engineering docs
stay in `ai/`. See `thesis/CLAUDE.md`. Its remote is the private GitHub repo
`AveryZC14/thesis`; push it only when the user asks.
