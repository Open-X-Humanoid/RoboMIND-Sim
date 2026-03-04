import traceback
import threading
from common.logger_loader import logger

class ZerorpcUtils(object):

    @staticmethod
    def start_server(robot_rpc, port):

        def _listener_rpc() -> None:
            try:
                import zerorpc
                s = zerorpc.Server(robot_rpc)
                s.bind(f"tcp://0.0.0.0:{port}")
                s.run()
            except (Exception,):
                logger.error('Error in DaemonLauncher._listener_rpc: %s' % traceback.format_exc())

        threading.Thread(target=_listener_rpc, name='RpcListener', daemon=True).start()


# def start_server(robot_rpc, port):

#     def _listener_rpc() -> None:
#         try:
#             import zerorpc
#             s = zerorpc.Server(robot_rpc)
#             s.bind(f"tcp://0.0.0.0:{port}")
#             s.run()
#         except (Exception,):
#             logger.error('Error in DaemonLauncher._listener_rpc: %s' % traceback.format_exc())

#     threading.Thread(target=_listener_rpc, name='RpcListener', daemon=True).start()