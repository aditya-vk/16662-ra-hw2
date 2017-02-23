import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        while True:
            sample = self.planning_env.GenerateRandomConfiguration()
            f_vid, f_vtx = ftree.GetNearestVertex(sample)
            r_vid, r_vtx = rtree.GetNearestVertex(sample)
            f_best = self.planning_env.Extend(f_vtx, sample)
            r_best = self.planning_env.Extend(r_vtx, sample)
            if f_best is not None:
                f_new_vid = ftree.AddVertex(f_best)
                ftree.AddEdge(f_vid, f_new_vid)
                self.planning_env.PlotEdge(f_vtx, f_best)
            if r_best is not None:
                r_new_vid = rtree.AddVertex(r_best)
                rtree.AddEdge(r_vid, r_new_vid)
                self.planning_env.PlotEdge(r_vtx, r_best)
            if f_best != None and r_best != None:
                dist = self.planning_env.ComputeDistance(f_best, r_best)
                if dist < epsilon:
                    # build plan here
                    cursor = f_new_vid
                    while(cursor != ftree.GetRootId()):
                        plan.append(ftree.vertices[cursor])
                        cursor = ftree.edges[cursor]
                    plan.append(ftree.vertices[cursor])
                    plan.reverse()
                    cursor = r_new_vid
                    while(cursor != rtree.GetRootId()):
                        cursor = rtree.edges[cursor]
                        plan.append(rtree.vertices[cursor])
                    break

        return plan
