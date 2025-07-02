# classifier.py

def classify_object_by_name(object_name):
    # Define lists of biodegradable and non-biodegradable object names
    # These lists are the core of your classification logic.
    biodegradable_items = [
        "banana",
        "apple",
        "cracker_box",
        "strawberry",
        "pear"
    ]

    non_biodegradable_items = [
        "tennis_ball",
        "mug",
        "soccer_ball",
        "toy_airplane",
        "bleach_cleanser"
    ]

    # Use if/elif/else statements to check the object's name against these lists
    if object_name in biodegradable_items:
        return "biodegradable"
    elif object_name in non_biodegradable_items:
        return "non-biodegradable"

# Example usage (for testing this file independently)
if __name__ == "__main__":
    print(f"Classifying 'banana': {classify_object_by_name('banana')}")
    print(f"Classifying 'mug': {classify_object_by_name('mug')}")
    print(f"Classifying 'apple': {classify_object_by_name('apple')}")
