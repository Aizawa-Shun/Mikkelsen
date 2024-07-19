import pybullet as p

def constrain_to_xz_plane(body_id):
    # In each simulation step, constrain the y position and velocity
    pos, orn = p.getBasePositionAndOrientation(body_id)
    pos[1] = 0
    p.resetBasePositionAndOrientation(body_id, pos, orn)
    
    linear_velocity, angular_velocity = p.getBaseVelocity(body_id)
    linear_velocity[1] = 0
    p.resetBaseVelocity(body_id, linearVelocity=linear_velocity, angularVelocity=angular_velocity)