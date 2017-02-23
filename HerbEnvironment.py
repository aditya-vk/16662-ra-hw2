import numpy
import math

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

        import numpy
        lower_limits = numpy.array(lower_limits)
        upper_limits = numpy.array(upper_limits)

        # Generate random configuration
        choice = numpy.random.rand(1)
        if choice < self.p:
            config = self.goal_config
        else:
            COLLISION = True
            while COLLISION:
                config = numpy.random.rand(len(self.robot.GetActiveDOFIndices()))*(upper_limits - lower_limits) + lower_limits
                # Check if it is collision free
                with self.robot:
                    robot_pos = self.robot.GetActiveDOFValues()
                    robot_pos = config
                    self.robot.SetActiveDOFValues(robot_pos)
                    if (self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()) == False:
                        COLLISION = False
        return numpy.array(config)


    
    def ComputeDistance(self, start_config, end_config):
        return numpy.linalg.norm(end_config - start_config)


    def Extend(self, start_config, end_config):
        epsilon = .01
        dist = self.ComputeDistance(start_config, end_config)
        numSteps = math.ceil(dist / epsilon)
        step = (end_config - start_config) / numSteps
        best_config = None
        i = 1
        while i <= numSteps:
            cur_config = start_config+step*i
            # Check if it is collision free
            with self.robot:
                robot_pos = self.robot.GetActiveDOFValues()
                robot_pos = cur_config
                self.robot.SetActiveDOFValues(robot_pos)
                if self.robot.GetEnv().CheckCollision(self.robot) == True:
                    return best_config
            #update variables
            best_config = cur_config
            i += 1
        return end_config
        
    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        return path
