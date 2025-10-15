import yaml
import json
import os
from pathlib import Path
from jsonschema import validate, ValidationError, Draft202012Validator


class VitisConfigManager:
    def __init__(self, config_path):
        self.config_path = Path(config_path)
        self.schema_path = Path("scripts/config_scheme.json")
        self.config = None
        self.errors = []

    def load_and_validate(self):
        # Load YAML config
        try:
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.errors.append(f"Failed to load YAML: {e}")
            return False

        # Load JSON schema
        try:
            with open(self.schema_path, 'r') as f:
                schema = json.load(f)
        except Exception as e:
            self.errors.append(f"Failed to load schema: {e}")
            return False

        # Validate against schema
        try:
            validate(instance=self.config, schema=schema,
                     cls=Draft202012Validator)
        except ValidationError as e:
            self.errors.append(f"Schema validation error: {e.message}")
            return False

        # Additional path checks
        self._check_paths()
        return len(self.errors) == 0

    def _check_paths(self):
        """Check that all required paths exist and do not end with a slash."""
        project = self.config.get("Project", {})
        path_fields = [
            "vitis_library_path",
            "opencv_include_path",
            "opencv_library_path"
        ]
        for field in path_fields:
            path = project.get(field)
            if path:
                if path.endswith('/'):
                    self.errors.append(
                        f"Path '{field}' should not end with a slash: {path}")
                if not Path(path).exists():
                    self.errors.append(
                        f"Path '{field}' does not exist: {path}")

        # Platform paths
        platform = project.get("platform", {})
        for field in ["petalinux_dir", "xsa_file_path"]:
            path = platform.get(field)
            if path:
                if path.endswith('/'):
                    self.errors.append(
                        f"Path '{field}' should not end with a slash: {path}")
                if not Path(path).exists():
                    self.errors.append(
                        f"Path '{field}' does not exist: {path}")

    def get(self, key_path, default=None):
        """Access config values using dot notation, e.g., 'Project.kernels.0.name'."""
        keys = key_path.split('.')
        value = self.config
        try:
            for key in keys:
                if isinstance(value, list):
                    key = int(key)
                value = value[key]
            return value
        except (KeyError, IndexError, ValueError, TypeError):
            self.errors.append(f"Malformed or missing value for '{key_path}'")
            return default

    def get_errors(self):
        return self.errors

    def is_valid(self):
        return len(self.errors) == 0
