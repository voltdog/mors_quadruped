"""Defines custom errors."""


class TemplateNotFoundError(RuntimeError):
    """Error raised when a template is not found."""

    def __init__(self, template_name: str) -> None:
        super().__init__(f"Template {template_name} not found.")


class TemplateDirectoryNotFoundError(RuntimeError):
    """Error raised when the template directory is not found."""

    def __init__(self) -> None:
        super().__init__("Template directory not found.")


class ModelValidationError(RuntimeError):
    """Error raised when a model is invalid."""

    def __init__(self, message: str) -> None:
        super().__init__(message)
