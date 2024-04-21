import json
from pathlib import Path
import unittest

import jsonschema

from timor import ModulesDB
from timor.utilities.file_locations import schema_dir
from timor.utilities.schema import get_schema_validator


class TestSchema(unittest.TestCase):
    """Tests properties and methods of the Module implementation."""
    def setUp(self) -> None:
        self.db_file = Path(__file__).parent.parent.joinpath("modules.json")

    def test_validation(self):
        _, validator = get_schema_validator(schema_dir.joinpath("ModuleSchema.json"))
        try:
            validator.validate(json.load(self.db_file.open("r")))
        except jsonschema.exceptions.ValidationError:
            self.fail(f"Failed to validate modules.json: "
                      f"{tuple(validator.iter_errors(json.load(self.db_file.open('r'))))}")

    def test_DB_init(self):
        ModulesDB.from_file(self.db_file, self.db_file.parent)


if __name__ == '__main__':
    unittest.main()
