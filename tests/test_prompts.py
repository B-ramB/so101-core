"""so101.prompts: yes/no and numbered-choice questions, with `input` replaced by scripted answers."""

import pytest

from so101.prompts import ask_choice, ask_yes_no


@pytest.fixture
def answers(monkeypatch):
    """Call answers("a", "b") to make the next input() calls return those strings. Returns the prompts shown."""
    prompts = []

    def script(*replies: str) -> list[str]:
        queue = list(replies)

        def fake_input(prompt: str = "") -> str:
            prompts.append(prompt)
            return queue.pop(0)

        monkeypatch.setattr("builtins.input", fake_input)
        return prompts

    return script


@pytest.mark.parametrize("default, suffix", [(True, "[Y/n]"), (False, "[y/N]"), (None, "[y/n]")])
def test_yes_no_shows_the_default(answers, default, suffix):
    prompts = answers("y")
    ask_yes_no("Go?", default=default)
    assert prompts == [f"Go? {suffix} "]


@pytest.mark.parametrize("default", [True, False])
def test_yes_no_empty_answer_returns_the_default(answers, default):
    answers("")
    assert ask_yes_no("Go?", default=default) is default


@pytest.mark.parametrize("reply, expected", [("y", True), ("YES", True), (" n ", False), ("No", False)])
def test_yes_no_answers(answers, reply, expected):
    answers(reply)
    assert ask_yes_no("Go?", default=not expected) is expected


def test_yes_no_asks_again_after_invalid_input(answers, capsys):
    prompts = answers("maybe", "", "y")  # without a default, an empty answer is invalid too
    assert ask_yes_no("Go?") is True
    assert len(prompts) == 3
    assert capsys.readouterr().out.count("Please answer 'y' or 'n'.") == 2


OPTIONS = ["leader_arm_2026-11-08.json", "leader_arm_2026-11-01.json", "leader_arm_2026-10-20.json"]


def test_choice_lists_the_options_and_marks_the_default(answers, capsys):
    answers("")
    ask_choice("Which?", OPTIONS, default=1)
    out = capsys.readouterr().out
    assert f"  [0] {OPTIONS[0]}\n" in out
    assert f"  [1] {OPTIONS[1]} (default)\n" in out


def test_choice_empty_answer_returns_the_default(answers):
    answers("")
    assert ask_choice("Which?", OPTIONS, default=2) == 2


def test_choice_by_number(answers):
    answers("1")
    assert ask_choice("Which?", OPTIONS) == 1


def test_choice_asks_again_after_invalid_input(answers, capsys):
    prompts = answers("3", "-1", "x", "s", "2")  # out of range, negative, not a number, skip not allowed
    assert ask_choice("Which?", OPTIONS) == 2
    assert len(prompts) == 5
    assert capsys.readouterr().out.count("Invalid choice.") == 4


def test_choice_skip(answers):
    prompts = answers("s")
    assert ask_choice("Which?", OPTIONS, allow_skip=True) is None
    assert prompts == ["Which? (Choose 0-2, 's' to skip): "]


def test_choice_without_skip_does_not_offer_it(answers):
    prompts = answers("0")
    ask_choice("Which?", OPTIONS)
    assert prompts == ["Which? (Choose 0-2): "]
