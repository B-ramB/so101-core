"""Small terminal input helpers for the interactive setup."""


def ask_yes_no(question: str, default: bool | None = None) -> bool:
    suffix = {True: " [Y/n] ", False: " [y/N] ", None: " [y/n] "}[default]
    while True:
        answer = input(question + suffix).strip().lower()
        if not answer and default is not None:
            return default
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("Please answer 'y' or 'n'.")


def ask_choice(question: str, options: list[str], default: int = 0, allow_skip: bool = False) -> int | None:
    """Let the user pick one option by number. Returns the index, or None when skipped."""
    for i, option in enumerate(options):
        marker = " (default)" if i == default else ""
        print(f"  [{i}] {option}{marker}")
    hint = f"Choose 0-{len(options) - 1}" + (", 's' to skip" if allow_skip else "")
    while True:
        answer = input(f"{question} ({hint}): ").strip().lower()
        if not answer:
            return default
        if allow_skip and answer == "s":
            return None
        if answer.isdigit() and int(answer) < len(options):
            return int(answer)
        print("Invalid choice.")


def header(title: str) -> None:
    print("\n" + "=" * 60 + f"\n {title}\n" + "=" * 60)
