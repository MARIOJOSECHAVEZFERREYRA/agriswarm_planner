"""
Pytest configuration for unit tests.

Mocks SQLAlchemy so that backend modules that declare ORM models can be
imported in unit tests without requiring a live database or sqlalchemy
to be installed in the test environment.

Only adds mocks for modules not already present in sys.modules, so this
is safe to use alongside integration tests that do use the real sqlalchemy.
"""

import importlib.util
import sys
from pathlib import Path
from unittest.mock import MagicMock


ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))


def pytest_configure(config):
    if importlib.util.find_spec("sqlalchemy") is not None:
        return
    sqla_mock = MagicMock()
    for mod in [
        "sqlalchemy",
        "sqlalchemy.orm",
        "sqlalchemy.ext",
        "sqlalchemy.ext.declarative",
    ]:
        if mod not in sys.modules:
            sys.modules[mod] = sqla_mock
