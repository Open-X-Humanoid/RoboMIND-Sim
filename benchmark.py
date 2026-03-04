#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Benchmark main entry file
Robot evaluation system based on Isaac Sim
"""

import argparse
import sys
import os
import json
from pathlib import Path
from typing import List, Optional

# Add project root directory to Python path
sys.path.append(str(Path(__file__).parent))

from common.logger_loader import logger
from common.config_loader import config_loader
from tasks.task_manager import TaskManager
# from models.model_interface import ModelInterface  # Model inference not implemented yet
# from results.result_logger import ResultLogger  # Temporarily commented, not testing yet


class BenchmarkRunner:
    """Benchmark evaluation system main controller"""

    def __init__(self):
        self.task_manager = None
        # self.model_interface = None  # Model inference not implemented yet
        # self.result_logger = None  # Temporarily commented, not testing yet

    def parse_arguments(self) -> argparse.Namespace:
        """Parse command line arguments"""
        parser = argparse.ArgumentParser(description='Isaac Sim robot evaluation system',
                                         formatter_class=argparse.RawDescriptionHelpFormatter,
                                         epilog="""
Example usage:
  python benchmark.py --task tasks/pick_apple.toml --model model1.pth,model2.pth --loop 10
  python benchmark.py --task tasks/pick_apple.toml --model model1.pth --loop 5 --headless
            """)

        parser.add_argument('--task', type=str, required=True, help='task name')

        # Model inference not implemented yet, commenting out model parameters
        # parser.add_argument(
        #     '--model',
        #     type=str,
        #     required=True,
        #     help='Model paths, multiple models separated by commas'
        # )

        parser.add_argument('--loop', type=int, default=1, help='Number of loop tests (default: 1)')

        parser.add_argument('--headless', action='store_true', help='Run in headless mode (no GUI)')

        parser.add_argument('--output', type=str, default='./logs', help='Result output directory (default: ./logs)')

        parser.add_argument('--zmq-port', type=int, default=5555, help='ZMQ communication port (default: 5555)')

        parser.add_argument('--verbose', action='store_true', help='Verbose log output')

        parser.add_argument('--timeout', type=int, default=300, help='Task execution timeout in seconds (default: 300)')

        return parser.parse_args()

    def validate_args(self, args: argparse.Namespace) -> bool:
        """Validate command line arguments"""
        # Validate task file exists
        if not config_loader.check_task_toml(args.task):
            logger.error(f"Task configuration file does not exist: {args.task}")
            return False

        # Model inference not implemented yet, commenting out model validation
        # model_paths = [path.strip() for path in args.model.split(',')]
        # for model_path in model_paths:
        #     if not os.path.exists(model_path):
        #         logger.error(f"Model file does not exist: {model_path}")
        #         return False

        # Validate loop count
        if args.loop <= 0:
            logger.error(f"Loop count must be greater than 0: {args.loop}")
            return False

        # Create output directory
        os.makedirs(args.output, exist_ok=True)

        return True

    def initialize_components(self, args: argparse.Namespace) -> bool:
        """Initialize components"""
        try:
            # Initialize result logger (temporarily commented)
            # self.result_logger = ResultLogger(args.output)

            # Initialize task manager (pass task name)
            # Extract task name from config file path
            self.task_manager = TaskManager(args.task)

            # Model interface not implemented yet
            # self.model_interface = ModelInterface(port=args.zmq_port)

            logger.info("Component initialization completed")
            return True

        except Exception as e:
            logger.error(f"Component initialization failed: {e}")
            return False

    def run_benchmark(self, args: argparse.Namespace) -> bool:
        """Run benchmark with success rate statistics"""
        logger.info(f"Starting benchmark - Task: {args.task}, Loop count: {args.loop}")

        success_count = 0
        total_count = 0
        successful_usd_paths = []  # Store USD paths of successful tasks

        for loop_idx in range(args.loop):
            logger.info(f"\n--- Test {loop_idx + 1}/{args.loop} ---")

            try:
                # Run task and get result
                success = self.task_manager.run_task(loop_idx=loop_idx, headless=args.headless, timeout=args.timeout, num_steps=60000)

                total_count += 1
                if success:
                    success_count += 1
                    # Read USD path from the status file actually used during this run
                    status_file_path = getattr(self.task_manager, 'last_status_file_path', None)
                    if not status_file_path:
                        # Fallback to constructing a new path (may mismatch timestamp)
                        status_file_path = self.task_manager._create_status_file_path(loop_idx)
                    usd_path = self._read_usd_path_from_status(status_file_path)
                    if usd_path:
                        successful_usd_paths.append(usd_path)

                logger.info(f"Test completed - Success: {success}")

            except Exception as e:
                logger.error(f"Test execution failed: {e}")
                total_count += 1  # Count failed attempts too
                continue
            finally:
                # Ensure task process is stopped
                self.task_manager.stop_task_process()

        # Calculate and display success rate
        success_rate = (success_count / total_count * 100) if total_count > 0 else 0.0

        logger.info("\n=== Benchmark Results ===")
        logger.info(f"Total tests: {total_count}")
        logger.info(f"Successful tests: {success_count}")
        logger.info(f"Failed tests: {total_count - success_count}")
        logger.info(f"Success rate: {success_rate:.1f}%")
        
        # Print successful task USD paths
        if successful_usd_paths:
            logger.info("\n=== Successful Task USD Paths ===")
            for i, usd_path in enumerate(successful_usd_paths, 1):
                logger.info(f"Success {i}: {usd_path}")
        else:
            logger.info("No successful tasks to display USD paths")
            
        logger.info("=== Benchmark completed ===")

        return True

    def _read_usd_path_from_status(self, status_file_path: str) -> Optional[str]:
        """Read environment_usd_path from task status file"""
        try:
            if os.path.exists(status_file_path):
                with open(status_file_path, 'r', encoding='utf-8') as f:
                    status_data = json.load(f)
                    return status_data.get('environment_usd_path')
            else:
                logger.warning(f"Status file not found: {status_file_path}")
                return None
        except Exception as e:
            logger.error(f"Failed to read USD path from status file {status_file_path}: {e}")
            return None

    def cleanup(self):
        """Clean up resources"""
        try:
            # Model interface not implemented yet
            # if self.model_interface:
            #     self.model_interface.cleanup()
            if self.task_manager:
                self.task_manager.cleanup()
            logger.info("Resource cleanup completed")
        except Exception as e:
            logger.error(f"Resource cleanup failed: {e}")

    def run(self) -> int:
        """Main run function"""
        try:
            # Parse arguments
            args = self.parse_arguments()
            logger.debug("parse_arguments done")
            # Validate arguments
            if not self.validate_args(args):
                return 1
            logger.debug("validate_arguments done")
            # Initialize components
            if not self.initialize_components(args):
                return 1
            logger.debug("initialize_components done")
            # Run benchmark
            if not self.run_benchmark(args):
                return 1
            logger.debug("run_benchmark done")
            return 0

        except KeyboardInterrupt:
            logger.info("\nUser interrupted benchmark")
            return 1
        except Exception as e:
            logger.error(f"Benchmark run failed: {e}")
            return 1
        finally:
            self.cleanup()


def main():
    """Main function"""
    runner = BenchmarkRunner()
    exit_code = runner.run()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
