import pybullet as p

class Camera:
    def __init__(self, dimension):
        self.dimension = dimension

    def set(self, focus, flag):
        ''' Adjust the camera position and focus '''
        if self.dimension == '3d':
            if flag:
                focus_position, _ = p.getBasePositionAndOrientation(focus)
                p.resetDebugVisualizerCamera(
                    cameraDistance=0.6, cameraYaw=50, cameraPitch=-10, cameraTargetPosition=focus_position
                )
            else:
                pass
        
        elif self.dimension == '2d':
            if flag:
                body_id = 4
                focus_position = p.getLinkState(focus, body_id)[4]
                p.resetDebugVisualizerCamera(
                    cameraDistance=0.6, cameraYaw=0, cameraPitch=0, cameraTargetPosition=focus_position
                )
            else:
                pass
        
        else:
            pass