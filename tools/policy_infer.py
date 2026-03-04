import zmq
import pickle
import cv2
import torch
import numpy as np
from pathlib import Path
# from lerobot.policies.act.modeling_act import ACTPolicy as Policy

class ZmqPublisher(object):
    def __init__(self, port, NUM_SNDHWM=1):
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, NUM_SNDHWM)
        self.socket.bind(f"tcp://127.0.0.1:{port}")
        print(f"ZmqPublisher bind to port {port}")

    def send_msg(self, data, topic):
        """Send message with optional topic
        
        Args:
            data: Data to send
            topic: Topic prefix (bytes)
        """
        message = topic + pickle.dumps(data)
        self.socket.send(message)
        print(f"send msg from zmq with topic: {topic}")

class ZmqReceiver(object):
    def __init__(self, port, NUM_RCVHWM=1):
        """Initialize ZMQ Receiver
        
        Args:
            port: Port number to connect to
            NUM_RCVHWM: High water mark for incoming messages
        """
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVHWM, NUM_RCVHWM)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all topics
        self.socket.connect(f"tcp://127.0.0.1:{port}")
        print(f"ZmqReceiver connected to port {port}, subscribing to all topics")
    
    def receive_msg(self, timeout=None):
        """Receive message from publisher
        
        Args:
            timeout: Timeout in milliseconds (None for blocking)
            
        Returns:
            Tuple of (topic, unpickled_data) or None if timeout
        """
        try:
            if timeout is not None:
                # Non-blocking receive with timeout
                if self.socket.poll(timeout) & zmq.POLLIN:
                    message = self.socket.recv(zmq.NOBLOCK)
                else:
                    return None
            else:
                # Blocking receive
                message = self.socket.recv()
                print(f"Received message of size: {len(message)} bytes")
            
            # Extract topic and data
            # The message format from ZmqPublisher is: topic + pickle.dumps(data)
            # We need to find where the topic ends and pickle data begins
            
            # Try known topics first
            known_topics = [b"obs", b"start", b"reset", b"test"]
            
            topic = b""
            data_bytes = message
            
            # Check for known topics
            for test_topic in known_topics:
                if message.startswith(test_topic):
                    try:
                        # Try to unpickle the remaining data
                        remaining_data = message[len(test_topic):]
                        data = pickle.loads(remaining_data)
                        topic = test_topic
                        print(f"Successfully parsed topic: {topic}, data size: {len(remaining_data)} bytes")
                        return (topic, data)
                    except Exception as parse_error:
                        print(f"Failed to parse data for topic {test_topic}: {parse_error}")
                        # Continue to try other topics instead of returning immediately
                        continue
            
            # If no known topic matched, try to find topic by searching for pickle header
            # Pickle data typically starts with \x80\x04 (protocol 4) or \x80\x03 (protocol 3)
            pickle_headers = [b'\x80\x04', b'\x80\x03', b'\x80\x02']
            
            for header in pickle_headers:
                header_pos = message.find(header)
                if header_pos > 0:  # Found pickle header after some bytes (topic)
                    topic = message[:header_pos]
                    remaining_data = message[header_pos:]
                    try:
                        data = pickle.loads(remaining_data)
                        print(f"Successfully parsed unknown topic: {topic}, data size: {len(remaining_data)} bytes")
                        return (topic, data)
                    except Exception as e:
                        print(f"Failed to parse data for unknown topic {topic}: {e}")
                        continue        
            
        except zmq.Again:
            # No message available
            return None
        except Exception as e:
            print(f"Error receiving message: {e}")
            print(f"Message preview: {message[:50] if 'message' in locals() else 'N/A'}")
            return None
    
    def close(self):
        """Close the socket"""
        self.socket.close()
        print("ZmqReceiver socket closed")


class PolicyInference:
    def __init__(self, model_path):
        self.model_path = Path(model_path)
        self.device = self._get_device()
        self.policy = self._load_policy()
        self.cnt = 0

    def _get_device(self):

        if torch.cuda.is_available():
            device = torch.device("cuda")
            print("GPU is available. Device set to:", device)
            return device
        else:
            print("GPU is not available.")

    def _load_policy(self):
        print(f"Loading model from: {self.model_path}")
        policy = Policy.from_pretrained(self.model_path)
        policy.eval()
        policy.to(self.device)
        return policy

    def reset(self):
        self.policy.reset() 
        
    def prepare_inference_obs(self, obs):
        inference_data = {}
        camera_names = ['camera']

        for cam_name in camera_names:
            cam_img = obs['images'][cam_name]
            # cam_img = cv2.imdecode(cam_img, cv2.IMREAD_COLOR)
            # cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
            cam_img = cv2.resize(cam_img, dsize=(640, 360))
            cam_img_tensor = torch.from_numpy(cam_img).permute(2, 0, 1).float() / 255.
            # / 255.0
            inference_data[f'observation.images.camera_{cam_name}'] = cam_img_tensor.unsqueeze(0).to(self.device, non_blocking=True)

        self.cnt += 1

        # 处理位置信息
        qpos = np.concatenate((obs['arm_joints']['robot'], obs['hand_joints']['left'], obs['hand_joints']['right']))
        qpos_data = torch.from_numpy(qpos).float()
        inference_data['observation.state'] = qpos_data.unsqueeze(0).to(self.device, non_blocking=True)

        return inference_data

    def infer(self, obs=None):
        input_data = self.prepare_inference_obs(obs)
        output_dict = self.policy.select_action(input_data)
        return output_dict

if __name__ == "__main__":
    # Model configuration
    # model_path = "/path/to/pretrained_model"
    
    # Initialize inference engine
    # inference_engine = PolicyInference(model_path)
    
    # Initialize ZMQ communication
    import time
    time.sleep(2)
    
    # Receiver for getting messages from simulation
    zmq_receiver = ZmqReceiver(port=5556)
    
    # Publisher for sending actions back to simulation
    zmq_publisher = ZmqPublisher(port=5557)
    
    print("Inference engine ready, waiting for simulation...")
    
    # State machine variables
    simulation_running = False
    
    robot = None

    try:
        while True:
            # Receive message with timeout to avoid blocking
            result = zmq_receiver.receive_msg()
            
            if result is None:
                continue  # No message received, continue waiting
            
            topic, data = result
            
            # Handle different message types
            if topic == b"start":
                print("Simulation initialization started...")
                simulation_running = True
                robot = data
                print(f"Robot: {robot}")
                # Reset inference engine for new episode
                # inference_engine.reset()
            elif topic == b"obs":
                if not simulation_running:
                    print("Warning: Received observation but simulation not started.")
                    continue
                # Check if data is None
                if data is None:
                    print("Error: Received obs topic but data is None, skipping...")
                    continue
                print("Received observation, performing inference...")
                # Extract camera image for debugging
                try:
                    if robot == b"TienKung":
                        head_bgr_img = data["camera_observations"]["color_images"]["camera_head"]
                        head_rgb_img = cv2.cvtColor(head_bgr_img, cv2.COLOR_BGR2RGB)
                        # Display image
                        cv2.imshow('Camera Head', head_rgb_img)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q') or key == 27: 
                            break
                except Exception as e:
                    print(f"Error extracting camera image: {e}")
                
                # For debugging communication only
                print("Observation received successfully, inference disabled for debugging")
                
            elif topic == b"reset":
                print("Simulation episode ended, resetting inference engine...")
                simulation_running = False
                # inference_engine.reset()
                print("Ready for next episode")
            elif topic == b"test":
                print("Infer recv func warmed up")
                zmq_publisher.send_msg(data=None,topic=b"test")
            else:
                print(f"Unknown topic received: {topic}")
                
    except KeyboardInterrupt:
        print("\nShutting down inference engine...")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Clean up
        zmq_receiver.close()
        cv2.destroyAllWindows()
        print("Inference engine shutdown complete")