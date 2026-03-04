import pickle
import zmq
from common.logger_loader import logger

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
            known_topics = [b"action",b"test"]
            
            topic = b""

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