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
            elif len(parts) == 5:
                generation = int(parts[0])
                min_fitness = float(parts[1])
                avg_fitness = float(parts[2])
                max_fitness = float(parts[3])
                stdev = float(parts[4])
                data.append((generation, min_fitness, avg_fitness, max_fitness, stdev))
    return data


single_recipes = {
    "ga-egg_toast-fixed-2025-12-10 03:58:09.782456-20-20-0.8-0.2/ga-egg_toast-fixed-2025-12-10 03:58:09.782456-20-20-0.8-0.2.log",
    "ga-pasta_with_tomato_sauce-fixed-2025-12-10 02:53:07.577585-20-20-0.8-0.2/ga-pasta_with_tomato_sauce-fixed-2025-12-10 02:53:07.577585-20-20-0.8-0.2.log",
    "ga-smoothie-fixed-2025-12-10 01:59:50.273967-20-20-0.8-0.2/ga-smoothie-fixed-2025-12-10 01:59:50.273967-20-20-0.8-0.2.log",
    "ga-pasta_with_tomato_sauce-distance-2025-12-10 02:38:24.413134-1-1-0.8-0.2/ga-pasta_with_tomato_sauce-distance-2025-12-10 02:38:24.413134-1-1-0.8-0.2.log",
    "ga-egg_toast-all-2025-12-10 03:06:36.527113-20-20-0.8-0.2/ga-egg_toast-all-2025-12-10 03:06:36.527113-20-20-0.8-0.2.log",
    "ga-pasta_with_tomato_sauce-all-2025-12-10 02:38:27.880258-20-20-0.8-0.2/ga-pasta_with_tomato_sauce-all-2025-12-10 02:38:27.880258-20-20-0.8-0.2.log",
    "ga-salad-fixed-2025-12-10 01:34:58.441836-20-20-0.8-0.2/ga-salad-fixed-2025-12-10 01:34:58.441836-20-20-0.8-0.2.log",
    "ga-smoothie-all-2025-12-10 01:47:30.042444-20-20-0.8-0.2/ga-smoothie-all-2025-12-10 01:47:30.042444-20-20-0.8-0.2.log",
    "ga-egg_toast-distance-2025-12-10 03:06:26.950295-1-1-0.8-0.2/ga-egg_toast-distance-2025-12-10 03:06:26.950295-1-1-0.8-0.2.log",
    "ga-smoothie-distance-2025-12-10 01:47:21.826211-1-1-0.8-0.2/ga-smoothie-distance-2025-12-10 01:47:21.826211-1-1-0.8-0.2.log",
    "ga-salad-distance-2025-12-10 01:21:50.283788-1-1-0.8-0.2/ga-salad-distance-2025-12-10 01:21:50.283788-1-1-0.8-0.2.log",
    "ga-toast_with_tomato-all-2025-12-10 02:11:38.254116-20-20-0.8-0.2/ga-toast_with_tomato-all-2025-12-10 02:11:38.254116-20-20-0.8-0.2.log",
    "ga-toast_with_tomato-fixed-2025-12-10 02:24:10.060734-20-20-0.8-0.2/ga-toast_with_tomato-fixed-2025-12-10 02:24:10.060734-20-20-0.8-0.2.log",
    "ga-salad-all-2025-12-10 01:21:56.693299-20-20-0.8-0.2/ga-salad-all-2025-12-10 01:21:56.693299-20-20-0.8-0.2.log",
    "ga-toast_with_tomato-distance-2025-12-10 02:11:29.711428-1-1-0.8-0.2/ga-toast_with_tomato-distance-2025-12-10 02:11:29.711428-1-1-0.8-0.2.log",
}

for string in single_recipes:
    print(f'RESULT_SINGLE_{string[string.find("ga-") + 3:string.find("-2025")].replace('-', '_').upper()} = ', end='')
    print(read_fitness_file(f'/ros2_ws/src/ga_runs/single recipes/{string}'))
print("")

combined_2 = {
    "ga-combined-2-retry-all-2025-12-10 19:22:44.900257-1-1-0.8-0.2/ga-combined-2-retry-all-2025-12-10 19:22:44.900257-1-1-0.8-0.2.log",
    "ga-combined-2-retry-distance-2025-12-10 19:22:49.041727-1-1-0.8-0.2/ga-combined-2-retry-distance-2025-12-10 19:22:49.041727-1-1-0.8-0.2.log",
    "ga-combined-2-retry-fixed-2025-12-10 19:22:53.165281-1-1-0.8-0.2/ga-combined-2-retry-fixed-2025-12-10 19:22:53.165281-1-1-0.8-0.2.log",
}

for string in combined_2:
    print(f'RESULT_{string[string.find("ga-") + 3:string.find("-2025")].replace('-', '_').upper()} = ', end='')
    print(read_fitness_file(f'/ros2_ws/src/ga_runs/retry results/{string}'))
print("")

combined_4 = {
    "ga-combined-4-retry-all-2025-12-10 19:22:57.177276-1-1-0.8-0.2/ga-combined-4-retry-all-2025-12-10 19:22:57.177276-1-1-0.8-0.2.log",
    "ga-combined-4-retry-distance-2025-12-10 19:23:07.031319-1-1-0.8-0.2/ga-combined-4-retry-distance-2025-12-10 19:23:07.031319-1-1-0.8-0.2.log",
}

for string in combined_4:
    print(f'RESULT_{string[string.find("ga-") + 3:string.find("-2025")].replace('-', '_').upper()} = ', end='')
    print(read_fitness_file(f'/ros2_ws/src/ga_runs/retry results/{string}'))
print("")

random_singles = {
    "ga-combined-2-random-all-2025-12-10 20:18:44.563996-20-1-0.8-0.2/ga-combined-2-random-all-2025-12-10 20:18:44.563996-20-1-0.8-0.2.log",
    "ga-combined-2-random-distance-2025-12-10 20:20:49.809962-20-1-0.8-0.2/ga-combined-2-random-distance-2025-12-10 20:20:49.809962-20-1-0.8-0.2.log",
    "ga-combined-2-random-fixed-2025-12-10 20:23:01.610981-20-1-0.8-0.2/ga-combined-2-random-fixed-2025-12-10 20:23:01.610981-20-1-0.8-0.2.log",
    "ga-combined-4-random-all-2025-12-10 20:25:11.602513-20-1-0.8-0.2/ga-combined-4-random-all-2025-12-10 20:25:11.602513-20-1-0.8-0.2.log",
    "ga-combined-4-random-distance-2025-12-10 20:29:32.538899-20-1-0.8-0.2/ga-combined-4-random-distance-2025-12-10 20:29:32.538899-20-1-0.8-0.2.log",
    "ga-egg_toast-random-all-2025-12-10 20:03:17.247036-20-1-0.8-0.2/ga-egg_toast-random-all-2025-12-10 20:03:17.247036-20-1-0.8-0.2.log",
    "ga-egg_toast-random-distance-2025-12-10 20:03:59.659686-20-1-0.8-0.2/ga-egg_toast-random-distance-2025-12-10 20:03:59.659686-20-1-0.8-0.2.log",
    "ga-egg_toast-random-fixed-2025-12-10 20:05:14.370773-20-1-0.8-0.2/ga-egg_toast-random-fixed-2025-12-10 20:05:14.370773-20-1-0.8-0.2.log",
    "ga-pasta_with_tomato_sauce-random-all-2025-12-10 20:06:26.453811-20-1-0.8-0.2/ga-pasta_with_tomato_sauce-random-all-2025-12-10 20:06:26.453811-20-1-0.8-0.2.log",
    "ga-pasta_with_tomato_sauce-random-distance-2025-12-10 20:07:15.374186-20-1-0.8-0.2/ga-pasta_with_tomato_sauce-random-distance-2025-12-10 20:07:15.374186-20-1-0.8-0.2.log",
    "ga-pasta_with_tomato_sauce-random-fixed-2025-12-10 20:08:22.158118-20-1-0.8-0.2/ga-pasta_with_tomato_sauce-random-fixed-2025-12-10 20:08:22.158118-20-1-0.8-0.2.log",
    "ga-salad-random-all-2025-12-10 20:09:39.798749-20-1-0.8-0.2/ga-salad-random-all-2025-12-10 20:09:39.798749-20-1-0.8-0.2.log",
    "ga-salad-random-distance-2025-12-10 20:10:44.940211-20-1-0.8-0.2/ga-salad-random-distance-2025-12-10 20:10:44.940211-20-1-0.8-0.2.log",
    "ga-salad-random-fixed-2025-12-10 20:11:48.203498-20-1-0.8-0.2/ga-salad-random-fixed-2025-12-10 20:11:48.203498-20-1-0.8-0.2.log",
    "ga-smoothie-random-all-2025-12-10 20:12:54.571909-20-1-0.8-0.2/ga-smoothie-random-all-2025-12-10 20:12:54.571909-20-1-0.8-0.2.log",
    "ga-smoothie-random-distance-2025-12-10 20:13:35.807669-20-1-0.8-0.2/ga-smoothie-random-distance-2025-12-10 20:13:35.807669-20-1-0.8-0.2.log",
    "ga-smoothie-random-fixed-2025-12-10 20:14:34.857911-20-1-0.8-0.2/ga-smoothie-random-fixed-2025-12-10 20:14:34.857911-20-1-0.8-0.2.log",
    "ga-toast_with_tomato-random-all-2025-12-10 20:15:35.750931-20-1-0.8-0.2/ga-toast_with_tomato-random-all-2025-12-10 20:15:35.750931-20-1-0.8-0.2.log",
    "ga-toast_with_tomato-random-distance-2025-12-10 20:16:42.986211-20-1-0.8-0.2/ga-toast_with_tomato-random-distance-2025-12-10 20:16:42.986211-20-1-0.8-0.2.log",
    "ga-toast_with_tomato-random-fixed-2025-12-10 20:17:47.763549-20-1-0.8-0.2/ga-toast_with_tomato-random-fixed-2025-12-10 20:17:47.763549-20-1-0.8-0.2.log",
}

for string in random_singles:
    print(f'RESULT_{string[string.find("ga-") + 3:string.find("-2025")].replace('-', '_').upper()} = ', end='')
    print(read_fitness_file(f'/ros2_ws/src/ga_runs/random level/{string}'))
print("")

partials = {
    "ga-combined-2-partials-2025-12-11 19:46:42.024310-1-1-0.8-0.2/ga-combined-2-partials-2025-12-11 19:46:42.024310-1-1-0.8-0.2.log",
    "ga-combined-2-partials-random-2025-12-11 20:02:25.614477-20-1-0.8-0.2/ga-combined-2-partials-random-2025-12-11 20:02:25.614477-20-1-0.8-0.2.log",
}

for string in partials:
    print(f'RESULT_{string[string.find("ga-") + 3:string.find("-2025")].replace('-', '_').upper()} = ', end='')
    print(read_fitness_file(f'/ros2_ws/src/ga_runs/partials/{string}'))
print("")
