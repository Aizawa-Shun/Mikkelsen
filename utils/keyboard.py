import pybullet as p

class InputHandler:
    def __init__(self):
        self.key_actions = {}

    def register_key_action(self, key, action):
        """Register an action to be performed when a specific key is released."""
        self.key_actions[key] = action

    def check_input(self):
        """Check for key inputs and perform registered actions."""
        input_keys = p.getKeyboardEvents()
        for key, action in self.key_actions.items():
            if input_keys.get(ord(key)) == p.KEY_WAS_RELEASED:
                action()
