import subprocess
import time
import os
import multiprocessing as mp
import signal
import sys
from typing import List
from multiprocessing import Process

class ProcessManager:
    def __init__(self):
        self.active_processes: List[Process] = []
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def add_processes(self, processes: List[Process]):
        self.active_processes.extend(processes)

    def cleanup(self):
        print("\nCleaning up processes...")
        for p in self.active_processes:
            if p.is_alive():
                print(f"Terminating process {p.pid}")
                p.terminate()
                p.join(timeout=3)
                
                if p.is_alive():
                    print(f"Force killing process {p.pid}")
                    p.kill()
                    p.join()
        print("All processes cleaned up")
        # Clear the list after cleanup
        self.active_processes.clear()

    def signal_handler(self, signum, frame):
        print(f"\nReceived signal {signum}")
        self.cleanup()
        sys.exit(0)

    def wait_for_processes(self):
        try:
            for p in self.active_processes:
                p.join()
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received")
            self.cleanup()
            sys.exit(0)


def run_roslaunch(package, launch_file, params):
    # Start roslaunch
    roslaunch = subprocess.Popen(['roslaunch', package, launch_file, *['{}:={}'.format(k, v) for k, v in params.items()]])

    # Wait for roslaunch to finish
    roslaunch.wait()

def eval_lego_assign(task):
    params = {
        'task': task,
        'use_rviz': 'false'
    }
    run_roslaunch('apex_mr', 'lego_assign.launch', params)

    time.sleep(2)


def eval_lego(ns, task, adg_t, sync_t, tight, biased, random, loop_type, sync, seed):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    apex_mr_dir = os.path.dirname(script_dir)
    directory = os.path.join(apex_mr_dir, 'outputs', 'lego', task)
        
    if not os.path.exists(directory):
        # If not, create the directory
        os.makedirs(directory)
        
    params = {
        'ns': ns,
        'task': task,
        'load_tpg': 'false',
        'load_adg': 'false',
        'benchmark': 'true',
        'use_rviz': 'false',
        'planner': 'RRTConnect',
        'planning_time_limit': 20,
        'adg_shortcut_time': str(adg_t),
        'sync_shortcut_time': str(sync_t),
        'tight_shortcut': 'true' if tight else 'false',
        'seed': str(seed),
        'sync_plan': 'true' if sync else 'false',
        'biased_sample': 'true' if biased else 'false',
        'random_shortcut': 'true' if random else 'false',
        'forward_doubleloop': 'true' if loop_type == 'fwd_diter' else 'false',
        'backward_doubleloop': 'true' if loop_type == 'bwd_diter' else 'false',
        'forward_singleloop': 'true' if loop_type == 'iter' else 'false',
        'progress_file': f"{directory}/progress_{seed}_{loop_type}{'_RRTConnect' if sync else ''}_{sync_t}_{'tight' if tight else 'loose'}{'_biased' if biased else ''}.csv",
    }

    print(params)
    run_roslaunch('apex_mr', f'lego.launch', params)

    time.sleep(2)

def add_lego_assign_process(task, id=0):
    processes = []

    id += 1
    p = mp.Process(target=eval_lego_assign, args=(task,))
    p.start()
    processes.append(p)
    time.sleep(1)

    return processes, id

def add_lego_processes(task, seeds, sync, max_time, id=0):
    processes = []
    biased = False
    random = True
    sync_t = 0.2 if sync else 0.1
    for seed in seeds:
        for tight in [True]:
            for loop_type in ['random']:
                adg_t = max_time if loop_type == 'random' else 0
                ns = f'run_{id}'
                id += 1
                p = mp.Process(target=eval_lego, args=(ns, task, adg_t, sync_t, tight, biased, random, loop_type, sync, seed))
                p.start()
                processes.append(p)
                time.sleep(1)
    return processes, id


if __name__ == "__main__":
    # Initialize process manager
    process_manager = ProcessManager()


    id = 0
    max_time = [20, 20, 20, 60, 60, 60, 60, 20, 60]
    seeds_set = [[0], [1], [2], [3]]
    tasks = ['cliff', 'stairs_rotated', 'faucet', 'bridge', 'fish_high', 'big_chair', 'vessel', 'guitar', 'rss']
    for task, max_t in zip(tasks, max_time):
        processes, id = add_lego_assign_process(task, id)
        process_manager.add_processes(processes)
        process_manager.wait_for_processes()

        for seeds in seeds_set:
            for sync in [True, False]:
                processes, id = add_lego_processes(task, seeds, sync, max_t, id)
                process_manager.add_processes(processes)
                process_manager.wait_for_processes()
 