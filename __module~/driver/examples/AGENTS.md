# AGENTS.md - Driver Examples

## Running Examples

Run an example from the driver crate root:

```bash
cargo run --example file_name
```

## Code Guidelines

- Examples may use `unwrap()` or `expect()` to stay concise.
- Examples should demonstrate the public API and avoid becoming alternate
  implementations of library logic.
