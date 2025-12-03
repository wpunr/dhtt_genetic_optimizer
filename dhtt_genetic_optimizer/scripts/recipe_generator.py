#!/usr/bin/python3

import os
import random
import yaml


def generate_yaml_combination(base_path, file_list, num_recipes, output_file="combined.yaml"):
    # Randomly select files with repetition allowed
    selected_files = [random.choice(file_list) for _ in range(num_recipes)]

    # Build NodeList: root AND node + all selected nodes
    yaml_node_list = ["GAExperimentAnd"] + [f"{os.path.splitext(f)[0]}_{i}" for i, f in enumerate(selected_files)]
    for i in range(len(yaml_node_list)):
        yaml_node_list[i] = yaml_node_list[i].replace('_', '-') # FixedPotential expects no underscores in the user-supplied name

    # Root AND node
    nodes = {
        "GAExperimentAnd": {
            "type": 1,
            "behavior_type": "dhtt_plugins::AndBehavior",
            "robot": 0,
            "parent": "NONE",
            "params": []
        }
    }

    # Add selected files as subtrees under GAExperimentAnd
    for idx, file_name in enumerate(selected_files):
        node_name = f"{os.path.splitext(file_name)[0]}_{idx}"
        node_name = node_name.replace('_', '-')
        nodes[node_name] = {
            "type": 5,
            "behavior_type": file_name,
            "robot": 0,
            "parent": "GAExperimentAnd",
            # "params": [f"mark: {chr(65 + (idx % 26))}#"]  # A#, B#, C#... wraps after Z
        }

    # Final YAML structure
    yaml_data = {
        "NodeList": yaml_node_list,
        "Nodes": nodes
    }

    # Write to output file
    with open(output_file, "w") as f:
        yaml.dump(yaml_data, f, sort_keys=False)

    # print(yaml.dump(yaml_data, sort_keys=False))

    print(f"✅ YAML combination created: {output_file}")
    print(f"Included files: {selected_files}")


# Example usage:
base_path = "/ros2_ws/src/dhtt_base/cooking_test/dhtt_cooking/test/experiment_descriptions"
file_list = ["recipe_egg_toast.yaml", "recipe_pasta_with_tomato_sauce.yaml", "recipe_salad.yaml",
             "recipe_smoothie.yaml", "recipe_toast_with_tomato.yaml", ]

for num_recipes in {2, 4, 8, 16, 20}:
    generate_yaml_combination(base_path, file_list, num_recipes, output_file=f'combined-{num_recipes}.yaml')
pass
