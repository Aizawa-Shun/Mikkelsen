import pybullet as p

def constrain_to_xz_plane(body_id):
    # In each simulation step, constrain the y position and velocity
    pos, orn = p.getBasePositionAndOrientation(body_id)
    new_pos = [pos[0], 0, pos[2]]  # Lock y position to 0
    p.resetBasePositionAndOrientation(body_id, new_pos, orn)
    
    linear_velocity, angular_velocity = p.getBaseVelocity(body_id)
    new_linear_velocity = [linear_velocity[0], 0, linear_velocity[2]]  # Lock y velocity to 0
    p.resetBaseVelocity(body_id, linearVelocity=new_linear_velocity, angularVelocity=angular_velocity)


