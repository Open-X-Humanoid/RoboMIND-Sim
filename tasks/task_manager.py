#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task Management Module
Responsible for task configuration reading, execution control, success determination, etc.
"""
import os
import sys
import time
import json
import numpy as np
import subprocess
import signal
import threading
from datetime import datetime
from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass
from pathlib import Path

# Define project root directory
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Add project root directory to Python path
sys.path.append(PROJECT_ROOT)
from common.logger_loader import logger
from common.config_loader import config_loader
from common.utils.usd_selector import select_random_usd_path



def create_log_entry(completed=False, success=False, score=0.0,
                    completion_step=0, failure_reason=None,
                    source="task_execution", usd_path=None, **kwargs):
    """Create standardized log entry"""
    entry = {
        'timestamp': datetime.now().isoformat(),
        'completed': completed,
        'success': success,
        'score': score,
        'completion_step': completion_step,
        'failure_reason': failure_reason,
        'source': source
    }
    if usd_path:
        entry['usd_path'] = usd_path
    entry.update(kwargs)  # Add extra fields like process_info
    return entry




class TaskManager:
    """Task Manager, responsible for task configuration, execution and evaluation"""

    def __init__(self, task_name: str):
        """
        Initialize task manager
        
        Args:
            task_name: Task name (e.g.: TienKung_task_03)
        """
        self.task_name = task_name
        # Process management
        self.task_process = None
        self.is_task_running = False
        self.process_monitor_thread = None
        self.log_file = None
        # Keep the path of the Isaac Sim log file so we can tail it on errors
        self.log_file_path = None
        # Batch timestamp for grouping logs from same batch
        self.batch_timestamp = None
        self.current_usd_path = None
        # Track the latest status file path used in a run
        self.last_status_file_path = None

    def init_batch_timestamp(self):
        """Initialize batch timestamp for grouping logs from same batch"""
        if self.batch_timestamp is None:
            self.batch_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    def start_task_process(self, num_steps: int = 60000, headless: bool = False, status_file_path: str = None) -> bool:
        """Start task process"""
        try:
            if self.is_task_running:
                logger.warning("Task process is already running")
                return True

            logger.info(f"Starting task process: {self.task_name}")

            # Build task startup script path
            run_script_path = os.path.join(PROJECT_ROOT, "tasks", "run_task.py")

            if not os.path.exists(run_script_path):
                logger.error(f"Task startup script does not exist: {run_script_path}")
                return False

            # Get Isaac Sim Python path
            isaac_python_path = config_loader.get_isaac_python_path()
            # Start subprocess
            cmd = [isaac_python_path, run_script_path, "--task", self.task_name, "--steps", str(num_steps)]
            if headless:
                cmd.append("--headless")
            if status_file_path:
                cmd.extend(["--status-file", status_file_path])
            if self.current_usd_path:
                cmd.extend(["--usd-path", self.current_usd_path])

            # Create timestamped log file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            # Ensure logs/isaac_sim directory exists
            logs_dir = os.path.join(PROJECT_ROOT, "logs", "isaac_sim")
            os.makedirs(logs_dir, exist_ok=True)

            log_file_path = os.path.join(logs_dir, f"isaac_sim_{self.task_name}_{timestamp}.log")
            log_file = open(log_file_path, "w")

            self.task_process = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=log_file,
                preexec_fn=os.setsid  # Create new process group
            )

            # Save log file reference for later closure
            self.log_file = log_file
            self.log_file_path = log_file_path
            logger.info(f"Isaac Sim log output to: {log_file_path}")

            self.is_task_running = True

            # Start process monitoring thread
            self.process_monitor_thread = threading.Thread(target=self._monitor_process, daemon=True)
            self.process_monitor_thread.start()

            logger.success(f"Task process started successfully, PID: {self.task_process.pid}")
            return True

        except Exception as e:
            logger.error(f"Failed to start task process: {e}")
            return False

    def stop_task_process(self) -> bool:
        """Stop task process"""
        try:
            if not self.is_task_running or not self.task_process:
                logger.info("Task process is not running")
                return True

            logger.info("Stopping task process...")

            # Send termination signal to process group
            try:
                os.killpg(os.getpgid(self.task_process.pid), signal.SIGTERM)

                # Wait for process to end
                try:
                    self.task_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    logger.warning("Process did not end within 10 seconds, force terminating")
                    os.killpg(os.getpgid(self.task_process.pid), signal.SIGKILL)
                    self.task_process.wait()

            except ProcessLookupError:
                logger.info("Process has ended")

            self.is_task_running = False
            self.task_process = None

            logger.success("Task process has been stopped")
            return True

        except Exception as e:
            logger.error(f"Failed to stop task process: {e}")
            return False

    def _monitor_process(self):
        """Monitor task process status"""
        try:
            while self.is_task_running and self.task_process:
                # Check if process is still running
                poll_result = self.task_process.poll()
                if poll_result is not None:
                    # Process has ended
                    self.is_task_running = False

                    if poll_result == 0:
                        logger.info("Task process ended normally")
                    else:
                        logger.warning(f"Task process ended abnormally, exit code: {poll_result}")
                        # Do not output Isaac Sim log content to console, only prompt log file path
                    break

                time.sleep(1)  # Check every second

        except Exception as e:
            logger.error(f"Process monitoring exception: {e}")
            self.is_task_running = False

    def _create_status_file_path(self, loop_idx: int) -> str:
        """Create status file path with batch folder structure: task_name_batch_timestamp/loop{idx}_timestamp.json"""
        # Ensure batch timestamp is initialized
        self.init_batch_timestamp()

        # Create folder: task_name_batch_timestamp (same for all loops in this batch)
        folder_name = f"{self.task_name}_{self.batch_timestamp}"
        status_dir = os.path.join(PROJECT_ROOT, "logs", "task_results", folder_name)
        os.makedirs(status_dir, exist_ok=True)

        # Create file: loop{idx}_timestamp.json (unique timestamp for each loop)
        file_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"loop{loop_idx}_{file_timestamp}.json"
        # Save and return
        self.last_status_file_path = os.path.join(status_dir, filename)
        return self.last_status_file_path

    def _read_task_status(self, status_file_path: str) -> dict:
        """Read task status from file"""
        try:
            if os.path.exists(status_file_path):
                with open(status_file_path, 'r') as f:
                    return json.load(f)
        except Exception as e:
            logger.warning(f"Failed to read task status file: {e}")

        # Return default status
        return {
            'completed': False,
            'success': False,
            'score': 0.0,
            'completion_step': 0,
            'failure_reason': None,
            'source': 'unknown'
        }

    def _write_log_entry(self, log_file_path: str, log_entry: dict):
        """Write log entry to file"""
        try:
            with open(log_file_path, 'w') as f:
                json.dump(log_entry, f, indent=2)
            logger.info(f"Log entry written to: {log_file_path}")
        except Exception as e:
            logger.error(f"Failed to write log entry: {e}")

    def run_task(self, loop_idx: int = 0, num_steps: int = 60000, headless: bool = False, timeout: int = 300) -> bool:
        """Execute task with three status handling: normal/timeout/failure"""
        start_time = time.time()
        log_file_path = self._create_status_file_path(loop_idx)

        # Select random USD path for this task execution
        config_loader.load_task_toml(self.task_name)
        usd_path_config = config_loader.task_config.get('environment', {}).get('usd_path')

        if usd_path_config:
            self.current_usd_path = select_random_usd_path(usd_path_config)
            logger.info(f"Selected USD environment: {self.current_usd_path}")
        else:
            self.current_usd_path = None
            logger.warning(f"No USD path configured for task: {self.task_name}")

        try:
            logger.info(f"Starting task execution: {self.task_name} (iteration {loop_idx + 1})")
            logger.info(f"Log file: {log_file_path}")

            # Start task process
            if not self.start_task_process(num_steps, headless, log_file_path):
                # Process failed to start
                execution_time = time.time() - start_time
                log_entry = create_log_entry(completed=False,
                                             success=False,
                                             score=0.0,
                                             completion_step=0,
                                             failure_reason="Failed to start task process",
                                             source="process_manager",
                                             usd_path=self.current_usd_path,
                                             process_info={
                                                 "exit_code": None,
                                                 "timeout": False,
                                                 "execution_time": execution_time,
                                                 "error_output": "Process startup failed"
                                             })
                self._write_log_entry(log_file_path, log_entry)

                return False

            # Wait for task completion or timeout (without IO operations in loop)
            while self.is_task_running and (time.time() - start_time) < timeout:
                time.sleep(1)

            execution_time = time.time() - start_time

            # Handle three cases: timeout, failure, normal
            if self.is_task_running:
                # Case 1: Timeout occurred
                logger.warning(f"Task execution timeout ({timeout}s), stopping process")
                self.stop_task_process()

                log_entry = create_log_entry(
                    completed=False,
                    success=False,
                    score=0.0,
                    completion_step=0,
                    failure_reason=
                    f"Process timeout after {timeout} seconds, task fails, please use run_task() to reproduce it",
                    source="process_manager",
                    usd_path=self.current_usd_path,
                    process_info={
                        "exit_code": None,
                        "timeout": True,
                        "execution_time": execution_time
                    })
                self._write_log_entry(log_file_path, log_entry)

                return False

            else:
                # Process ended, check exit code
                exit_code = self.task_process.poll() if self.task_process else None

                if exit_code != 0 and exit_code is not None:
                    # Case 2: Process failed (abnormal exit)
                    logger.error(f"Task process failed with exit code: {exit_code}")

                    log_entry = create_log_entry(completed=False,
                                                 success=False,
                                                 score=0.0,
                                                 completion_step=0,
                                                 failure_reason=f"Process crashed with exit code {exit_code}",
                                                 source="process_manager",
                                                 usd_path=self.current_usd_path,
                                                 process_info={
                                                     "exit_code": exit_code,
                                                     "timeout": False,
                                                     "execution_time": execution_time,
                                                     "error_output": "Process abnormal termination"
                                                 })
                    self._write_log_entry(log_file_path, log_entry)

                    return False

                else:
                    # Case 3: Normal completion - read task result from file
                    task_status = self._read_task_status(log_file_path)
                    logger.info(f"Task execution completed normally, duration: {execution_time:.2f}s")
                    logger.info(f"Task success: {task_status.get('success', False)}")
                    return task_status.get('success', False)

        except Exception as e:
            # Exception handling
            execution_time = time.time() - start_time
            logger.error(f"Task execution exception: {e}")

            # Ensure process is stopped
            self.stop_task_process()

            log_entry = create_log_entry(completed=False,
                                         success=False,
                                         score=0.0,
                                         completion_step=0,
                                         failure_reason=f"Exception occurred: {str(e)}",
                                         source="process_manager",
                                         usd_path=self.current_usd_path,
                                         process_info={
                                             "exit_code": None,
                                             "timeout": False,
                                             "execution_time": execution_time,
                                             "error_output": str(e)
                                         })
            self._write_log_entry(log_file_path, log_entry)

            return False

    def is_process_running(self) -> bool:
        """Check if task process is running"""
        return self.is_task_running and self.task_process is not None

    def cleanup(self) -> bool:
        """Clean up task manager"""
        try:
            logger.info("Cleaning up task manager...")

            # Stop task process
            if self.is_task_running:
                self.stop_task_process()

            # Wait for monitoring thread to end
            if self.process_monitor_thread and self.process_monitor_thread.is_alive():
                self.process_monitor_thread.join(timeout=5)

            # Close log file
            if self.log_file:
                try:
                    self.log_file.close()
                    logger.info("Log file closed")
                except Exception as e:
                    logger.warning(f"Failed to close log file: {e}")

            # Reset state
            self.task_process = None
            self.is_task_running = False
            self.process_monitor_thread = None
            self.log_file = None
            self.log_file_path = None

            logger.success("Task manager cleanup completed")
            return True

        except Exception as e:
            logger.error(f"Failed to clean up task manager: {e}")
            return False
