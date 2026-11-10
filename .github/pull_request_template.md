## Summary
<!-- What the change does, in two or three sentences. -->

## Why
<!-- The problem it solves. If it came from a vehicle, say which and what was seen. -->

## Details
<!-- Design choices, limits, anything a reviewer should look at closely. -->

## Depends on
<!-- Pull requests this one is stacked on, or "none". -->

## Docs
<!-- Pages updated: docs/manual.md, docs/at_commands.md, docs/diagnostics.md, docs/developers.md, README.md. -->

## Testing
- [ ] Host tests pass (`ctest --test-dir test/host/build`), with new tests for this change
- [ ] Firmware builds (`idf.py build` or `docker compose run --rm --build firmware`)
- [ ] `mkdocs build --strict` passes
- [ ] Code formatted with `tools/format.sh`

**To confirm on hardware:**
<!-- What has not been tried on a real vehicle or adapter yet. -->
