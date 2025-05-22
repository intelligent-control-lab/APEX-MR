#!/usr/bin/env python3
import rospy
import json
import os.path as osp
from apex_mr.srv import StabilityScore, StabilityScoreResponse
from stability_analysis_with_support  import stability_score_with_support
from stability_analysis_graph_input import stability_score
from utils import load_json

class StabilityServer:
    def __init__(self, path):
        self.lego_library = load_json(osp.join(path, "lego_library.json"))
        self.config = load_json(osp.join(path, "config.json"))
        self.config['Visualize_Analysis'] = 0

        s = rospy.Service('stability_score', StabilityScore, self.handle_stability_score)
        rospy.loginfo("Ready to compute stability score with support")

    def handle_stability_score(self, req):
        # Assuming your function is defined elsewhere and imported

        lego_structure = load_json(req.lego_structure_path)
        use_support = req.use_support
        if use_support:
            print(lego_structure)
            support_x = req.support_x
            support_y = req.support_y
            support_z = req.support_z
            support_ori = req.support_ori 
            print(support_x, support_y, support_z, support_ori)
            analysis_score, num_vars, num_constr, total_t, solve_t  = \
                stability_score_with_support(lego_structure, support_x, support_y, support_z, support_ori, self.lego_library, self.config)
        else:
            analysis_score, num_vars, num_constr, total_t, solve_t = \
                stability_score(lego_structure, self.lego_library, self.config)

        violation = analysis_score[analysis_score > 0.99]

        res = StabilityScoreResponse()
        res.stable = (len(violation) <= 0)
        return res

def stability_score_server():
    rospy.init_node('stability_score_server')
    # get the path to the assembly_sequence_planning package
    path = osp.dirname(osp.abspath(__file__))
    
    server = StabilityServer(path)
    rospy.spin()

if __name__ == "__main__":
    stability_score_server()