#!/usr/bin/env python3

import json
import csv
import copy
import os
from typing import Dict, List, Tuple

class TaskProcessor:
    def __init__(self, base_folder: str):
        """
        Initialize TaskProcessor with base folder path.
        
        Args:
            base_folder: Root folder path (e.g., /path/to/my_folder)
        """
        self.base_folder = base_folder
        self.seq_data: Dict = {}
        self.env_data: Dict = {}
        self.steps_data: List[List] = []
        self.steps_per_task = 13  # Default steps per task (except first task with 12)

    def load_files(self, task_name: str) -> None:
        """Load the input files for given task name."""
        # Define file paths
        seq_file = os.path.join(self.base_folder, 'steps', f'{task_name}_seq.json')
        env_file = os.path.join(self.base_folder, 'env_setup', f'env_setup_{task_name}.json')
        steps_file = os.path.join(self.base_folder, 'steps', f'{task_name}_steps.csv')
        
        # Load sequence JSON
        with open(seq_file, 'r') as f:
            self.seq_data = json.load(f)
        
        # Load environment setup JSON
        with open(env_file, 'r') as f:
            self.env_data = json.load(f)
            
        # Load steps CSV
        with open(steps_file, 'r') as f:
            csv_reader = csv.reader(f)
            self.steps_data = [row for row in csv_reader]

    def create_subset_task(self, task_name: str, start_task: int, end_task: int) -> None:
        """
        Create new task files from start_task to end_task and save them to appropriate directories.
        
        Args:
            task_name: Base name of the task (e.g., 'stairs')
            start_task: Starting task number (1-based)
            end_task: Ending task number (1-based)
        """
        # Validate input
        if start_task < 1 or end_task > len(self.seq_data):
            raise ValueError("Invalid task range")
        
        # Generate new task name
        new_task_name = f"{task_name}_{start_task}_{end_task}"
        
        # Update environment based on tasks before start_task
        updated_env = self._update_environment(start_task - 1)
        
        # Create new sequence data with selected tasks
        new_seq = {}
        task_counter = 1
        for i in range(start_task, end_task + 1):
            new_seq[str(task_counter)] = copy.deepcopy(self.seq_data[str(i)])
            task_counter += 1
            
        # Extract corresponding steps
        new_steps = self._extract_steps(start_task, end_task)
        
        # Define output paths
        seq_output = os.path.join(self.base_folder, 'steps', f'{new_task_name}_seq.json')
        env_output = os.path.join(self.base_folder, 'env_setup', f'env_setup_{new_task_name}.json')
        steps_output = os.path.join(self.base_folder, 'steps', f'{new_task_name}_steps.csv')
        
        # Ensure directories exist
        os.makedirs(os.path.dirname(seq_output), exist_ok=True)
        os.makedirs(os.path.dirname(env_output), exist_ok=True)
        
        # Write files
        with open(seq_output, 'w') as f:
            json.dump(new_seq, f, indent=4)
            
        with open(env_output, 'w') as f:
            json.dump(updated_env, f, indent=4)
            
        with open(steps_output, 'w', newline='') as f:
            csv_writer = csv.writer(f)
            csv_writer.writerows(new_steps)

    def _update_environment(self, up_to_task: int) -> Dict:
        """Update environment setup based on completed tasks."""
        updated_env = copy.deepcopy(self.env_data)
        
        # Process each task up to up_to_task
        for i in range(1, up_to_task + 1):
            task = self.seq_data[str(i)]
            brick_id = task['brick_id']
            brick_seq = task['brick_seq']
            
            # Create or update brick entry
            brick_key = f"b{brick_id}_{brick_seq}"
            updated_env[brick_key] = {
                'x': task['x'],
                'y': task['y'],
                'z': task['z'],
                'ori': task['ori'],
                'fixed': True
            }
            
        return updated_env

    def _extract_steps(self, start_task: int, end_task: int) -> List[List]:
        """Extract steps for the selected task range."""
        new_steps = []
        # always take the first step to home poses
        new_steps.append(self.steps_data[0])
        
        # Calculate start and end indices in steps data
        start_idx = 1
        for idx in range(1, start_task):
            manip_type = 0
            if 'manipulate_type' in self.seq_data[str(idx)]:
                manip_type = self.seq_data[str(idx)]['manipulate_type']
            if manip_type == 0:
                start_idx += 16
            else:
                start_idx += 27
        
        end_idx = 1
        for idx in range(1, end_task+1):
            manip_type = 0
            if 'manipulate_type' in self.seq_data[str(idx)]:
                manip_type = self.seq_data[str(idx)]['manipulate_type']
            if manip_type == 0:
                end_idx += 16
            else:
                end_idx += 27
        
        # Extract relevant steps

        new_steps += self.steps_data[start_idx:end_idx]
        
        return new_steps

def create_task_subset(task_name: str, start_task: int, end_task: int) -> None:
    """
    Main function to create a subset of tasks with updated environment.
    
    Args:
        task_name: Name of the task (e.g., 'stairs')
        start_task: Starting task number (1-based)
        end_task: Ending task number (1-based)
    """
    # Get the base folder path (assuming this script is in my_folder/src/exe/python)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    base_folder = os.path.abspath(os.path.join(current_dir, '..', '..', 'config', 'lego_tasks'))
    
    processor = TaskProcessor(base_folder)
    processor.load_files(task_name)
    processor.create_subset_task(task_name, start_task, end_task)

if __name__ == "__main__":
    # Example usage with argparse
    import argparse
    parser = argparse.ArgumentParser(description='Create subset of tasks')
    parser.add_argument('--task_name', type=str, help='Name of the task (e.g., stairs)', default='stairs')
    parser.add_argument('--start_task', type=int, help='Starting task number (1-based)', default=9)
    parser.add_argument('--end_task', type=int, help='Ending task number (1-based)', default=10)
    args = parser.parse_args()

    create_task_subset(args.task_name, args.start_task, args.end_task)