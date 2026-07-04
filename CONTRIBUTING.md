# Contributing

## Development Setup

```bash
git clone https://github.com/MahediIslamNadim/ground-station-gui.git
cd ground-station-gui
pip install -r requirements.txt
pip install ruff pre-commit
pre-commit install
```

## Code Style

- Ruff for linting and formatting (`ruff check . && ruff format .`)
- Type hints for all public functions
- Google-style docstrings for classes and public methods

## Pull Request Process

1. Ensure all tests pass: `python -m tests.test_mavlink`
2. Run the linter: `ruff check src/ tests/`
3. Update CHANGELOG.md with your changes
4. Open a PR against the `master` branch

## Testing

Tests are in the `tests/` directory and use Python's built-in `unittest`:

```bash
python -m tests.test_mavlink
```
