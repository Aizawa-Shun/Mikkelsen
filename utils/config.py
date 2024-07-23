import yaml

def load_config(file_path):
    """
    Function to read a configuration file and return it in dictionary format.
    
    Args: 
        file_path (str): Path of the configuration file.
        file_path (str): Path of the configuration file.
    
    Returns: dict
        dict: A dictionary containing the configuration.
    """
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)