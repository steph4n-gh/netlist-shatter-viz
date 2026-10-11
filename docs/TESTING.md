# Running the included checks

Install with `./scripts/install-alpha.sh`, then from the source root run:

```sh
cargo test --locked --all-targets -- --test-threads=1
PYTHONPATH=tests .venv/bin/python -m unittest discover -s tests -p 'test_*.py'
node tests/test_native_resume_ui.js
node tests/test_native_upload_ui.js
```

Node is only needed for these two UI regression scripts. Serialize the Rust
suite and run Python separately: some legacy tests assert short wall-clock
budgets and are sensitive to competing CPU work. These tests check their stated
properties; they do not qualify arbitrary boards or promise routing speed.

For a real native check, follow the LED example in [Quickstart](QUICKSTART.md).
Read the resulting job report, native findings, source preservation and CAM
results. Hardware remains physically unvalidated. The public test set excludes
historical experiments requiring development-only job directories and archives.
