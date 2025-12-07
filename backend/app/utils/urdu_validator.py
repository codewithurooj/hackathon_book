"""
Urdu Text Validation Utility

Validates that submitted text contains actual Urdu characters to prevent spam
and ensure translation quality.
"""

import re
from typing import Tuple


# Urdu Unicode range: U+0600 to U+06FF (Arabic script used for Urdu)
# Extended range: U+0600-U+06FF, U+0750-U+077F, U+FB50-U+FDFF, U+FE70-U+FEFF
URDU_UNICODE_PATTERN = re.compile(r'[\u0600-\u06FF\u0750-\u077F\uFB50-\uFDFF\uFE70-\uFEFF]')


def is_valid_urdu_text(text: str, min_urdu_percentage: float = 0.5) -> Tuple[bool, str]:
    """
    Validate that text contains a minimum percentage of Urdu characters.

    Args:
        text: The text to validate
        min_urdu_percentage: Minimum percentage of Urdu characters required (default: 0.5 = 50%)

    Returns:
        Tuple of (is_valid: bool, error_message: str)
        - If valid: (True, "")
        - If invalid: (False, "Error description")

    Examples:
        >>> is_valid_urdu_text("یہ اردو متن ہے")
        (True, "")

        >>> is_valid_urdu_text("This is English text")
        (False, "Text must contain at least 50% Urdu characters. Found: 0%")

        >>> is_valid_urdu_text("")
        (False, "Text cannot be empty")
    """
    # Check for empty text
    if not text or not text.strip():
        return False, "Text cannot be empty"

    # Count total characters (excluding whitespace)
    total_chars = len(text.replace(" ", "").replace("\n", "").replace("\t", ""))

    if total_chars == 0:
        return False, "Text contains only whitespace"

    # Count Urdu characters
    urdu_chars = len(URDU_UNICODE_PATTERN.findall(text))

    # Calculate percentage
    urdu_percentage = urdu_chars / total_chars

    # Validate percentage
    if urdu_percentage < min_urdu_percentage:
        return False, (
            f"Text must contain at least {int(min_urdu_percentage * 100)}% Urdu characters. "
            f"Found: {int(urdu_percentage * 100)}%"
        )

    return True, ""


def count_urdu_characters(text: str) -> dict:
    """
    Count Urdu characters and provide statistics.

    Args:
        text: The text to analyze

    Returns:
        Dictionary with:
        - total_chars: Total characters (excluding whitespace)
        - urdu_chars: Count of Urdu characters
        - urdu_percentage: Percentage of Urdu characters (0.0 to 1.0)
        - is_valid: Whether text meets 50% threshold

    Example:
        >>> count_urdu_characters("یہ اردو ہے with some English")
        {
            'total_chars': 25,
            'urdu_chars': 10,
            'urdu_percentage': 0.4,
            'is_valid': False
        }
    """
    total_chars = len(text.replace(" ", "").replace("\n", "").replace("\t", ""))
    urdu_chars = len(URDU_UNICODE_PATTERN.findall(text))

    urdu_percentage = urdu_chars / total_chars if total_chars > 0 else 0.0

    return {
        "total_chars": total_chars,
        "urdu_chars": urdu_chars,
        "urdu_percentage": urdu_percentage,
        "is_valid": urdu_percentage >= 0.5
    }


# Example usage and testing
if __name__ == "__main__":
    # Test cases
    test_cases = [
        ("یہ اردو متن ہے", True),  # Pure Urdu - should pass
        ("This is English", False),  # Pure English - should fail
        ("یہ mixed ہے text", None),  # Mixed - depends on ratio
        ("", False),  # Empty - should fail
        ("   ", False),  # Whitespace only - should fail
    ]

    print("Urdu Validator Test Results:")
    print("-" * 60)

    for text, expected in test_cases:
        is_valid, error = is_valid_urdu_text(text)
        stats = count_urdu_characters(text)

        print(f"Text: '{text[:30]}...' " if len(text) > 30 else f"Text: '{text}'")
        print(f"  Valid: {is_valid}")
        print(f"  Urdu %: {stats['urdu_percentage']:.1%}")
        if error:
            print(f"  Error: {error}")
        print()
