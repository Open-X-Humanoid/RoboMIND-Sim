"""
USD Path Selection Utility

This module provides functionality to randomly select USD files from environment directories.
"""

import os
import random
from common.logger_loader import logger
from common.config_loader import config_loader


def select_random_usd_path(environment_path: str = None) -> str:
    """
    Select a random USD file from the specified environment path.
    
    Args:
        environment_path: Path to the environment directory. If None, uses config default.
        
    Returns:
        str: Absolute path to the selected USD file
    """
    # Use provided path or fall back to config default
    env_path = environment_path if environment_path else config_loader.task_config['environment']['usd_path']

    # Get project root directory
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
    abs_environment_path = os.path.join(project_root, env_path)

    # If it's a directory, randomly select a USD file from it
    if os.path.isdir(abs_environment_path):
        usd_files = [f for f in os.listdir(abs_environment_path) if f.endswith('.usd')]
        if usd_files:
            chosen_usd = random.choice(usd_files)
            abs_environment_path = os.path.join(abs_environment_path, chosen_usd)
            logger.debug(f"Randomly selected USD file: {chosen_usd}")
        else:
            logger.warning(f"No USD files found in directory: {abs_environment_path}")

    logger.debug(f"Selected environment path: {abs_environment_path}")
    return abs_environment_path
