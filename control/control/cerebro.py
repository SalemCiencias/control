import rclpy
from rclpy.node import Node
from cui_interfaces.srv import SoundRequest
from action_cheese.action import Cheese
from action_doa.action import DOAaction

from . import manejoClientes 
import os
import subprocess
from std_msgs.msg import String

class salemVoiceClient(manejoClientes.ClientAsync):

    def build_request(self,bandera,texto):
        self.req.bandera = bandera
        self.req.texto = texto
    

class ControlNode(Node):

    def __init__(self):
        super().__init__('ControlNode')
        #Servicio de voz de salem
        self.voz_salem = salemVoiceClient('Voz_Salem', SoundRequest, 'sound_request')
        self.publisher_ = self.create_publisher(String, 'control_log', 10)

    def print_log(self,log_msg = ''):
        msg = String()
        msg.data = f"SAlEM: {log_msg}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def go_salem(self):
        '''
        Aqui se establecerá el flujo de control principal de
        la rutina de ejecución de salem.
        '''  
        # DOA girar al encontrar a la persona
        #self.audio_salem = manejoClientes.ActionClientManager('doa_action_client', DOAaction, 'doaaction')
        #self.audio_salem.send_goal(0)

        self.voz_salem.build_request('g','sounds/presentar')
        self.voz_salem.send_request()

        self.voz_salem.build_request('g','sounds/posar')
        self.voz_salem.send_request()
        
        # subprocess.Popen('python3  src/cheese_action_server.py 0', shell=True)
        # subprocess.Popen('python3  src/cheese_action_client.py', shell=True)


def main(args=None):
    rclpy.init(args=args)

    salem_brain = ControlNode()
    salem_brain.go_salem()
    rclpy.spin(salem_brain)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    salem_brain.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()