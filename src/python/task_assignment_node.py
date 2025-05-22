#!/usr/bin/env python3
import rospy
import json
import os.path as osp
import time
from apex_mr.srv import TaskAssignment, TaskAssignmentResponse
from task_assignment  import TaskAssignmentSolver
from utils import load_json

class TaskAssignmentNode:
    def __init__(self):
        s = rospy.Service('task_assignment', TaskAssignment, self.callback)
        self.solver = TaskAssignmentSolver()
        rospy.loginfo("Ready to compute MILP task assignment")

    def callback(self, req):
        # Assuming your function is defined elsewhere and imported

        cost_matrix, delta_matrix, s, s_j, precedence_matrix = self.solver.load_matrix(req.output_dir)
        n = cost_matrix.shape[0]
        m = cost_matrix.shape[1]
        p = cost_matrix.shape[2]
        g = cost_matrix.shape[3]
        #print(n, m, p, g, cost_matrix.shape, s.shape, delta_matrix.shape, s_j.shape, precedence_matrix)
        ts = time.time()
        assignment, status = self.solver.solve_problem(n, m, p, g, cost_matrix, s, delta_matrix, s_j, precedence_matrix)
        print(f"Status: {status}, Time: {time.time() - ts}")
        
        res = TaskAssignmentResponse()
        if status != 'Infeasible':

            with open(req.task_config_path, 'r') as f:
                task_config = json.load(f)

            self.solver.parse_solution(task_config, assignment)
            
            with open(req.output_dir + f"/{req.output_fname}", 'w') as f:
                json.dump(task_config, f, indent=4)
        
            res.feasible = True
        else:
            res.feasible = False

        return res
    

def task_assignment_server():
    rospy.init_node('milp_node')
      
    server = TaskAssignmentNode()
    rospy.spin()

if __name__ == "__main__":
    task_assignment_server()