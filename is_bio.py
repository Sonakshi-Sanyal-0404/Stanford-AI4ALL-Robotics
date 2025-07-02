def classify_by_coordinates(object_position):

    # Define sets of coordinates for classification
    # These coordinates should match the initial positions of your objects
    # in the PyBullet simulation.
    biodegradable_coords = {
        (1.0, 1.0, 0.5), # banana
        (2.0, 0.0, 0.5), # apple
        (3.0, 1.0, 0.5), # cracker_box
        (1.0, -1.0, 0.5), # strawberry
        (3.0, -1.0, 0.5), # pear
    }

    non_biodegradable_coords = {
        (0.0, 2.0, 0.5), # tennis_ball
        (2.0, 2.0, 0.5), # mug
        (0.0, -2.0, 0.5), # soccer_ball
        (2.0, -2.0, 0.5), # toy_airplane
        (2.0, -1.0, 0.5), # bleach_cleanser
    }

    # Round the input object_position to 1 decimal place for consistent comparison
    # with the predefined coordinate sets.
    rounded_position = tuple(round(coord, 1) for coord in object_position)

    # --- Classification using if statements based on coordinates ---
    if rounded_position in biodegradable_coords:
        return "biodegradable"
    elif rounded_position in non_biodegradable_coords:
        return "non-biodegradable"
    else:
        return "unknown"

# Example usage (for testing this file independently)
if __name__ == "__main__":
    # Test with coordinates that should classify as biodegradable
    print(f"Position (1.0, 1.0, 0.5) is: {classify_by_coordinates((1.0, 1.0, 0.5))}")
    print(f"Position (3.0, 1.0, 0.5) is: {classify_by_coordinates((3.0, 1.0, 0.5))}")
    print(f"Position (2.0, 0.0, 0.5) is: {classify_by_coordinates((2.0, 0.0, 0.5))}")
    print(f"Position (1.0, -1.0, 0.5) is: {classify_by_coordinates((1.0, -1.0, 0.5))}")
    print(f"Position (3.0, -1.0, 0.5) is: {classify_by_coordinates((3.0, -1.0, 0.5))}")

    # Test with coordinates that should classify as non-biodegradable
    print(f"Position (0.0, 2.0, 0.5) is: {classify_by_coordinates((0.0, 2.0, 0.5))}")
    print(f"Position (2.0, 2.0, 0.5) is: {classify_by_coordinates((2.0, 2.0, 0.5))}")
    print(f"Position (0.0, -2.0, 0.5) is: {classify_by_coordinates((0.0, -2.0, 0.5))}")
    print(f"Position (2.0, -2.0, 0.5) is: {classify_by_coordinates((2.0, -2.0, 0.5))}")
    print(f"Position (2.0, -1.0, 0.5) is: {classify_by_coordinates((2.0, -1.0, 0.5))}")