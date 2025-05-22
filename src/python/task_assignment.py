#!/usr/bin/env python3

import pulp
import numpy as np
import pandas as pd
import json
import time

class TaskAssignmentSolver:
    def __init__(self):
        self.inf = 1000000
        self.lambda_balance = 1000  # Weight factor for balancing
        self.block_names = []
        self.num_offsets = 7

    def load_matrix(self, out_dir):
        self.block_names = []
        self.block_reusable = []
        with open(out_dir + '/cost_matrix_a.csv', 'r') as f:
            header = f.readline()
            blocks = header.split(',')[:-1]
            t = len(blocks)
            for i in range(t):
                block_name, g = blocks[i].split('@')
                if block_name not in self.block_names:
                    self.block_names.append(block_name)
                    # check if the block name is from a station
                    if "station" in block_name:
                        self.block_reusable.append(True)
                    else:
                        self.block_reusable.append(False)
        num_blocks = len(self.block_names)

        # read csv file cost_matri_a.csv
        cost_matrix_a = np.genfromtxt(out_dir + '/cost_matrix_a.csv', delimiter=',', skip_header=1)
        if cost_matrix_a.ndim == 1:
            cost_matrix_a = cost_matrix_a.reshape(1, -1)
        cost_matrix_a = cost_matrix_a[:, :t]
    
        # read csv file time_matrix_a.csv
        cost_matrix_b = np.genfromtxt(out_dir + '/cost_matrix_b.csv', delimiter=',', skip_header=1)
        if cost_matrix_b.ndim == 1:
            cost_matrix_b = cost_matrix_b.reshape(1, -1)
        cost_matrix_b = cost_matrix_b[:, :t]

        num_tasks = len(cost_matrix_a)
        # read delta_matrix
        delta_matrix = np.genfromtxt(out_dir + '/delta_matrix.csv', delimiter=',', skip_header=1)
        if delta_matrix.ndim == 1:
            delta_matrix = delta_matrix.reshape(1, -1)
        delta_matrix = delta_matrix[:, :-1]
        # read support matrix
        s_a = np.genfromtxt(out_dir + '/support_matrix_a.csv', delimiter=',', skip_header=1)
        if s_a.ndim == 1:
            s_a = s_a.reshape(1, -1)
        s_a = s_a[:, :-1]
        
        s_b = np.genfromtxt(out_dir + '/support_matrix_b.csv', delimiter=',', skip_header=1)
        if s_b.ndim == 1:
            s_b = s_b.reshape(1, -1)
        s_b = s_b[:, :-1]

        num_grasps = s_a.shape[1]

        # read s_j
        s_j = np.genfromtxt(out_dir + '/support_req.csv', skip_header=1, delimiter=',')
        if s_j.ndim == 1:
            s_j = s_j.reshape(1, -1)
        s_j = s_j[:, :-1]

        # read precedence matrix
        precedence_matrix = np.genfromtxt(out_dir + '/precedence.csv', delimiter=',')
        if len(precedence_matrix) > 0:
            precedence_matrix = precedence_matrix[:, :2]

        cost_matrix_a = np.reshape(cost_matrix_a, (num_tasks, num_blocks, num_grasps))
        cost_matrix_b = np.reshape(cost_matrix_b, (num_tasks, num_blocks, num_grasps))
        cost_matrix = np.array([cost_matrix_a, cost_matrix_b])
        s = np.array([s_a, s_b])
        s_j = np.transpose(s_j)

        return cost_matrix, delta_matrix, s, s_j, precedence_matrix

    def solve_problem(self, n, m, p, q, c, s, delta, s_j, precedence_constraints):
        """
        n: number of robots
        m: number of tasks
        p: number of blocks
        q: number of grasp sides
        c: cost matrix
        s: support matrix
        delta: delta matrix (whether block type is compatible with task)
        s_j: support required matrix
        precedence_constraints: list of tuples (t1, t2) where t1 must be used before t2
        """
        if (n > m):
            print("Number of tasks must be greater than or equal to the number of robots")
            return None

        # Define the problem
        self.prob = pulp.LpProblem("Task_Assignment", pulp.LpMinimize)

        # Define the variables
        x = pulp.LpVariable.dicts("x", (range(n), range(m), range(p), range(q)), cat='Binary')
        y = pulp.LpVariable.dicts("y", (range(n), range(m), range(q)), cat='Binary')
        z = pulp.LpVariable.dicts("z", (range(n), range(m-n+1)), lowBound=0, cat='Integer')
        z_max = pulp.LpVariable.dicts("z_max", (range(m-n+1)), lowBound=0, cat='Integer')
        z_min = pulp.LpVariable.dicts("z_min", (range(m-n+1)), lowBound=0, cat='Integer')

        # Objective function
        self.prob += pulp.lpSum(c[i][j][t][g] * x[i][j][t][g] for i in range(n) for j in range(m) for t in range(p) for g in range(q)) + \
                    pulp.lpSum(s[i][j][g] * y[i][j][g] for i in range(n) for j in range(m) for g in range(q)) + \
                    self.lambda_balance * pulp.lpSum(z_max[k] - z_min[k] for k in range(m-n+1))

        # Constraints
        # Each task has one robot and one block
        for j in range(m):
            self.prob += pulp.lpSum(x[i][j][t][g] for i in range(n) for t in range(p) for g in range(q)) == 1

        # A robot cannot be assigned to both pick-and-place and support tasks for the same task
        for i in range(n):
            for j in range(m):
                self.prob += pulp.lpSum(x[i][j][t][g] for t in range(p) for g in range(q)) + pulp.lpSum(y[i][j][g] for g in range(q)) <= 1

        # If a task needs support, a robot is assigned to support
        for j in range(m):
            self.prob += pulp.lpSum(y[i][j][g] for i in range(n) for g in range(q)) == np.max(s_j[:, j])

        # Ensures the correct block type is chosen for each task
        for j in range(m):
            self.prob += pulp.lpSum(x[i][j][t][g] * delta[j][t] for i in range(n) for t in range(p) for g in range(q)) == 1

        for t in range(p):
            if not self.block_reusable[t]:
                # Each block is used at most once
                self.prob += pulp.lpSum(x[i][j][t][g] for i in range(n) for j in range(m) for g in range(q)) <= 1
        
        # the grasp side is selected for both pick-and-place and support robot
        for j in range(m):
            for g in range(q):
                self.prob += pulp.lpSum(x[i][j][t][g] for i in range(n) for t in range(p)) * np.max(s_j[:, j]) == pulp.lpSum(y[i][j][g] for i in range(n))

        # Precedence constraints
        for (t1, t2) in precedence_constraints:
            self.prob += pulp.lpSum(x[i][j][t1][g] * (m - j) for i in range(n) for j in range(m) for g in range(q)) >= \
                    pulp.lpSum(x[i][j][t2][g] * (m - j) for i in range(n) for j in range(m) for g in range(q))

        # Workload balancing constraints in a window of size n
        for i in range(n):
            for k in range(m-n+1):
                self.prob += z[i][k] == (pulp.lpSum(x[i][j][t][g] for t in range(p) for j in range(k, k+n) for g in range(q)) \
                                        + pulp.lpSum(y[i][j][g] for j in range(k, k+n) for g in range(q)))

        
        for i in range(n):
            for k in range(m-n+1):
                self.prob += z_min[k] <= z[i][k]
                self.prob += z_max[k] >= z[i][k]

        # Solve the problem
        self.prob.solve()
        solution = {}
        for v in self.prob.variables():
            solution[v.name] = v.varValue
        
        assignment = []
        for j in range(m):
            robot_idx = -1
            sup_idx = -1
            block_idx = -1
            press_side = -1
            press_offset = -1
            for i in range(n):
                if (robot_idx == -1):
                    for t in range(p):
                        for g in range(q):
                            if (robot_idx == -1) and (solution[f"x_{i}_{j}_{t}_{g}"] == 1):
                                robot_idx = i
                                block_idx = t
                                press_side = int(g // self.num_offsets) + 1
                                press_offset = g % self.num_offsets
            for i in range(n):
                for g in range(q):            
                    if s_j[robot_idx, j] and (sup_idx == -1):
                        if solution[f"y_{i}_{j}_{g}"] == 1:
                            sup_idx = i
            assignment.append((j, robot_idx, block_idx, press_side, press_offset, sup_idx))

        solution['Total Cost'] = pulp.value(self.prob.objective)
        if (solution['Total Cost'] > self.inf):
            return assignment, 'Infeasible'
        return assignment, pulp.LpStatus[self.prob.status]
    
    def parse_solution(self, config_json, assignment):
        for a in assignment:
            print(f"Task {a[0]} assigned to robot {a[1]+1} and lego block {a[2]} {self.block_names[a[2]]} " + \
                f"press_side {a[3]}, offset {a[4]}, support robot {a[5]+1}")
    
            task_id = f'{a[0]+1}'
            brick_seq = self.block_names[a[2]].split('_')[1]
            if brick_seq.isdigit():
                # the brick seq is a number, meaning it is in a unique location
                config_json[task_id]['brick_seq'] = int(brick_seq)
            else:
                # the brick seq is a string, meaning it is in a station, we add a unique task id to it
                config_json[task_id]['brick_seq'] = f'{brick_seq}.{task_id}'
            config_json[task_id]['press_side'] = a[3]
            config_json[task_id]['press_offset'] = a[4]
            config_json[task_id]['robot_id'] = a[1]+1
            config_json[task_id]['sup_robot_id'] = a[5]+1

