#!/usr/bin/env python3
"""
Task startup script
Used to start specified tasks through the Registry system
"""

import sys
import os
import argparse
from pathlib import Path

# Add project root directory to Python path
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_path not in sys.path:
    sys.path.append(project_path)

from isaacsim.simulation_app import SimulationApp


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Run specified task')
    parser.add_argument('--task', type=str, required=True, help='Task name')
    parser.add_argument('--steps', type=int, default=60000, help='Simulation steps')
    parser.add_argument('--headless', default=False, action='store_true', help='Run in headless mode')
    parser.add_argument('--status-file', type=str, help='Path to status file for task communication')
    parser.add_argument('--usd-path', type=str, help='USD environment path to use for the task')
    return parser.parse_args()


def main():
    """Main function"""
    try:
        args = parse_arguments()
        # logger.info(f"Starting task: {args.task}, steps: {args.steps}, headless: {args.headless}")

        # Configure Isaac Sim
        config = {
            "width": 1280,
            "height": 720,
            "sync_loads": False,
            "headless": bool(args.headless),
            "renderer": "RayTracedLighting",
            "anti_aliasing": 3,
        }
        # logger.info(f"Isaac Sim configuration: {config}")
        # Start SimulationApp
        simulation_app = SimulationApp(config)
        from common.logger_loader import logger
        from common.config_loader import config_loader
        from common.x_registry import Registry
        # logger.info("Isaac Sim startup completed")

        # Import task modules (must be after SimulationApp startup)
        import robots
        import tasks.TienKung_tasks

        config_loader.load_task_toml(args.task)
        task_config = config_loader.task_config

        # Prepare task parameters
        # Use command line USD path if provided, otherwise use config
        environment_path = getattr(args, 'usd_path', None)
        task_params = {
            'simulation_app': simulation_app,
            'environment_path': environment_path,
            'robot_init_position': task_config.get('robot', {}).get('init_position'),
            'status_file_path': getattr(args, 'status_file', None)
        }

        task_instance = Registry.create(args.task, **task_params)
        logger.info(f"Task instance created successfully: {type(task_instance).__name__}")
        if task_instance is None:
            logger.error(f"Unable to create task instance: {args.task}")
            return 1

        logger.info(f"Task instance created successfully: {type(task_instance).__name__}")

        # Start task
        task_instance.start()

        # # Run simulation
        task_instance.play(num_steps=args.steps)

        logger.success(f"Task {args.task} completed")
        return 0

    except Exception as e:
        logger.error(f"Task execution failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        # Ensure SimulationApp is properly closed
        try:
            simulation_app.close()
        except:
            pass


if __name__ == "__main__":
    sys.exit(main())
