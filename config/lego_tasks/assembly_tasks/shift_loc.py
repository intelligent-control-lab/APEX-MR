import json
import os

def change_file(task, dx, dy, dz):
    # Load the JSON file
    base_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(base_dir, f'{task}.json')
    with open(file_path, 'r') as file:
        data = json.load(file)

    # save the original JSON file to a new file
    output_path = os.path.join(base_dir, f'{task}_original.json')
    with open(output_path, 'w') as file:
        json.dump(data, file, indent=4)

    for key in data:
        data[key]["x"] += dx
        data[key]["y"] += dy
        data[key]["z"] += dz

        data[key]["press_x"] += dx
        data[key]["press_y"] += dy
        data[key]["press_z"] += dz

        if data[key]["support_x"] != -1:
            data[key]["support_x"] += dx
            data[key]["support_y"] += dy
            data[key]["support_z"] += dz
        # remove the brick_seq entry
        data[key].pop("brick_seq", None)

    # Save the modified JSON back to a file
    output_path = os.path.join(base_dir, f'{task}.json')
    with open(output_path, 'w') as file:
        json.dump(data, file, indent=4)

    print(f"Updated {task}.json file")

if __name__ == "__main__":
    task = 'bridge'
    dx, dy, dz = 0, 4, 0
    change_file(task, dx, dy, dz)