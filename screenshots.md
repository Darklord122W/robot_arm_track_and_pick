# Screenshot location — for Claude

This file tells Claude where to find screenshots when the user references one without giving an explicit path.

## Where they live

```
/home/darklord/Pictures/Screenshots/
```

That is the default Ubuntu / GNOME `Print Screen` save location for `darklord@darklord`. Every screenshot the user takes lands there.

## Filename convention

GNOME's default pattern:

```
Screenshot from YYYY-MM-DD HH-MM-SS.png
```

(Note the **space** between the date and time, and `HH-MM-SS` with hyphens, not colons.)

Examples already in the folder:

```
Screenshot from 2025-09-25 19-25-04.png
Screenshot from 2025-10-17 17-16-51.png
Screenshot from 2025-10-18 13-43-11.png
```

## How Claude should resolve a reference

When the user says things like *"see the screenshot"*, *"look at what I just took"*, *"the rqt window"*, or *"this error"* without a path, follow this order:

1. List the directory, sorted by mtime descending — the most recent file is almost always what the user means:
   ```bash
   ls -lt /home/darklord/Pictures/Screenshots/ | head -5
   ```
2. If the user mentioned a topic ("the depth one", "yesterday's"), filter by date in the filename or by `mtime`.
3. Read the file with the `Read` tool — it accepts PNG/JPG and renders the image visually.
4. If multiple recent shots exist and the choice is ambiguous, ask which one before reading more than the top hit (saves tokens, avoids guessing).

Quoting paths with `"Screenshot from ..."` requires double quotes because of the space:

```bash
ls "/home/darklord/Pictures/Screenshots/Screenshot from 2025-10-18 13-43-11.png"
```

## Why this file exists

The user asked for a persistent pointer in the workspace so any Claude session — even one that didn't load this conversation's memory — can find screenshots by reading this file. The same fact is also stored in Claude's auto memory as a reference-type entry, but this on-disk copy is the source of truth and lives with the project.
