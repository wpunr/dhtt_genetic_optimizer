def read_fitness_file(filepath):
    """
    Reads a file with rows in the format: generation,min_fitness,avg_fitness
    (no header) and returns a list of tuples.

    Parameters
    ----------
    filepath : str
        Path to the file.

    Returns
    -------
    list[tuple[int, float, float]]
        List of tuples: (generation, min_fitness, avg_fitness)
    """
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            # Strip whitespace and split by comma
            parts = line.strip().split(',')
            if len(parts) == 3:
                generation = int(parts[0])
                min_fitness = float(parts[1])
                avg_fitness = float(parts[2])
                data.append((generation, min_fitness, avg_fitness))
    return data


single_recipes = {
    "/ga-egg_toast-all-2025-11-26 21:51:46.795213-20-20-0.8-0.2/ga-egg_toast-all-2025-11-26 21:51:46.795213-20-20-0.8-0.2.log",
    "/ga-egg_toast-distance-2025-11-26 21:34:16.824883-1-1-0.8-0.2/ga-egg_toast-distance-2025-11-26 21:34:16.824883-1-1-0.8-0.2.log",
    "/ga-egg_toast-fixed-2025-11-26 22:08:58.823203-20-20-0.8-0.2/ga-egg_toast-fixed-2025-11-26 22:08:58.823203-20-20-0.8-0.2.log",
    "/ga-pasta_with_tomato_sauce-all-2025-11-26 22:28:10.216818-20-20-0.8-0.2/ga-pasta_with_tomato_sauce-all-2025-11-26 22:28:10.216818-20-20-0.8-0.2.log",
    "/ga-pasta_with_tomato_sauce-distance-2025-11-26 21:34:27.797556-1-1-0.8-0.2/ga-pasta_with_tomato_sauce-distance-2025-11-26 21:34:27.797556-1-1-0.8-0.2.log",
    "/ga-pasta_with_tomato_sauce-fixed-2025-11-26 22:46:02.770138-20-20-0.8-0.2/ga-pasta_with_tomato_sauce-fixed-2025-11-26 22:46:02.770138-20-20-0.8-0.2.log",
    "/ga-salad-all-2025-11-26 23:05:47.589383-20-20-0.8-0.2/ga-salad-all-2025-11-26 23:05:47.589383-20-20-0.8-0.2.log",
    "/ga-salad-distance-2025-11-26 21:34:37.075976-1-1-0.8-0.2/ga-salad-distance-2025-11-26 21:34:37.075976-1-1-0.8-0.2.log",
    "/ga-salad-fixed-2025-11-26 23:23:49.536196-20-20-0.8-0.2/ga-salad-fixed-2025-11-26 23:23:49.536196-20-20-0.8-0.2.log",
    "/ga-smoothie-all-2025-11-26 23:42:10.070546-20-20-0.8-0.2/ga-smoothie-all-2025-11-26 23:42:10.070546-20-20-0.8-0.2.log",
    "/ga-smoothie-distance-2025-11-26 21:34:46.202635-1-1-0.8-0.2/ga-smoothie-distance-2025-11-26 21:34:46.202635-1-1-0.8-0.2.log",
    "/ga-smoothie-fixed-2025-11-26 23:53:53.334798-20-20-0.8-0.2/ga-smoothie-fixed-2025-11-26 23:53:53.334798-20-20-0.8-0.2.log",
    "/ga-toast_with_tomato-all-2025-11-27 00:10:57.557010-20-20-0.8-0.2/ga-toast_with_tomato-all-2025-11-27 00:10:57.557010-20-20-0.8-0.2.log",
    "/ga-toast_with_tomato-distance-2025-11-26 21:34:59.461070-1-1-0.8-0.2/ga-toast_with_tomato-distance-2025-11-26 21:34:59.461070-1-1-0.8-0.2.log",
    "/ga-toast_with_tomato-fixed-2025-11-27 00:26:36.785162-20-20-0.8-0.2/ga-toast_with_tomato-fixed-2025-11-27 00:26:36.785162-20-20-0.8-0.2.log",
}

for string in single_recipes:
    print(f'RESULT_SINGLE_{string[string.find("ga-") + 3:string.find("-2025")].replace('-', '_').upper()} = ', end='')
    print(read_fitness_file(f'/ros2_ws/src/ga_runs/single recipes{string}'))

combined_2 = {
    "/ga-combined-2-fixed-2025-11-27 06:00:46.918797-20-20-0.8-0.2/ga-combined-2-fixed-2025-11-27 06:00:46.918797-20-20-0.8-0.2.log",
    "/ga-combined-2-distance-2025-11-27 05:32:01.400268-1-1-0.8-0.2/ga-combined-2-distance-2025-11-27 05:32:01.400268-1-1-0.8-0.2.log",
    "/ga-combined-2-resource-2025-11-27 05:32:05.170259-1-1-0.8-0.2/ga-combined-2-resource-2025-11-27 05:32:05.170259-1-1-0.8-0.2.log",
    "/ga-combined-2-all2-2025-11-27 20:13:25.702666-100-20-0.8-0.2/ga-combined-2-all-2025-11-27 20:13:25.702666-100-20-0.8-0.2.log",
    "/ga-combined-2-fixed-2025-11-27 22:09:50.297393-100-20-0.8-0.2/ga-combined-2-fixed-2025-11-27 22:09:50.297393-100-20-0.8-0.2.log",
    "/ga-combined-2-all-2025-11-27 05:35:53.062762-20-20-0.8-0.2/ga-combined-2-all-2025-11-27 05:35:53.062762-20-20-0.8-0.2.log",
}

for string in combined_2:
    print(f'RESULT_{string[string.find("ga-") + 3:string.find("-2025")].replace('-', '_').upper()} = ', end='')
    print(read_fitness_file(f'/ros2_ws/src/ga_runs/combined-2{string}'))

combined_4 = {
    "/ga-combined-4-resource-2025-11-27 05:33:06.720893-1-1-0.8-0.2/ga-combined-4-resource-2025-11-27 05:33:06.720893-1-1-0.8-0.2.log",
    "/ga-combined-4-all-2025-11-28 07:05:36.777525-100-20-0.8-0.2/ga-combined-4-all-2025-11-28 07:05:36.777525-100-20-0.8-0.2.log",
    "/ga-combined-4-distance-2025-11-27 05:32:50.844267-1-1-0.8-0.2/ga-combined-4-distance-2025-11-27 05:32:50.844267-1-1-0.8-0.2.log",
}

for string in combined_4:
    print(f'RESULT_{string[string.find("ga-") + 3:string.find("-2025")].replace('-', '_').upper()} = ', end='')
    print(read_fitness_file(f'/ros2_ws/src/ga_runs/combined-4{string}'))