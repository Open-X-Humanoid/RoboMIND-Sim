# import argparse
import os
# from typing import Any

import json
import tomli as tomllib
from pathlib import Path
from common.logger_loader import logger


class _ConfigLoader:

    def __init__(self) -> None:
        self.common_directory = Path(__file__).parent

    def load_robot_toml(self, robot_name):
        local_config_path_name = os.path.join(self.common_directory, 'robot_config', f'cfg_{robot_name}.toml')
        with open(local_config_path_name, 'rb') as local_f:
            self.robot_config = tomllib.load(local_f)
        logger.info(f'Robot configure file {robot_name} loaded.')

    def load_task_toml(self, task_name):
        local_config_path_name = os.path.join(self.common_directory, 'task_config', f'cfg_{task_name}.toml')
        with open(local_config_path_name, 'rb') as local_f:
            self.task_config = tomllib.load(local_f)
        logger.info(f'Task configure file {task_name} loaded.')
    
    def check_task_toml(self, task_name):
        task_toml_path = os.path.join(self.common_directory, 'task_config', f'cfg_{task_name}.toml')
        if not os.path.exists(task_toml_path):
            logger.error(f"Task configure file {task_name} not exists.")
            return False
        return True

    def get_isaac_python_path(self):
        """Get Isaac Sim Python path
        """
        try:
            isaac_config_path = os.path.join(self.common_directory, 'isaac_config.toml')
            if os.path.exists(isaac_config_path):
                with open(isaac_config_path, 'rb') as f:
                    isaac_config = tomllib.load(f)
                config_path = isaac_config.get('isaac_sim', {}).get('python_path')
                if config_path and os.path.exists(config_path):
                    logger.info(f'Using Isaac Python path from config file: {config_path}')
                    return config_path
                else:
                    logger.warning(f'Path in config file does not exist: {config_path}')
        except Exception as e:
            logger.warning(f'Failed to read Isaac config file: {e}')
         
        raise FileNotFoundError(
            "Unable to find Isaac Sim Python path. Please configure isaac_config.toml file"
        )


config_loader = _ConfigLoader()
