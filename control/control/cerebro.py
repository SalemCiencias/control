import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


from cui_interfaces.srv import SoundRequest
from action_cheese.action import Cheese

from .manejoClientes import ActionClientManager, ClientAsync
import os
import sys
from subprocess import Popen


from std_msgs.msg import String

CHEESE_CAMERA = int(sys.argv[1])

class CheeseClient(ActionClientManager):

    ALL_OK = 'YES'
    def __init__(self,node,action_type, action_name, cui_service):
        group = ReentrantCallbackGroup()
        self.client = ActionClient(node, action_type, action_name,callback_group=group)
        self.cui = cui_service
        self.node = node
        self.__goal_future = None
        self.__result_future = None

    def build_goal_msg(self,*args):
        order = args[0]
        goal_msg = Cheese.Goal()
        goal_msg.order = order
        return goal_msg

    def result_callback(self, future):
        result = future.result().result.result
        print(result)
        if result == self.ALL_OK:
            self.lock = False
            Popen(f'ros2 run face_recognition_pub talker {CHEESE_CAMERA}',shell=True)
            print('aqui ************************************************************')
            self.cui.build_request(
            'n', 'Prueba de que funicona')
            self.cui.send_request()

            print("Seguimos con la ejecucion")

            self.cui.build_request(
                'n', 'otra cosa')
            self.cui.send_request()
        else:
            print('Se intentará tomar de nuevo las fotos')
            self.sendlock_thread_goal('inicia')
        
        

    def feedback_callback(self, feedback_msg):
        self.cuenta += 1
        feedback = feedback_msg.feedback
        message = 'Received feedback: {0}'.format(feedback.progress)
        lock_thread = True if self.cuenta == 5 else False
        print(message)

class SalemVoiceClient(ClientAsync):

    def build_request(self, bandera, texto):
        self.req.bandera = bandera
        self.req.texto = texto


class ControlNode(Node):

    def __init__(self):
        super().__init__('ControlNode')
        self.voz_salem = SalemVoiceClient(
            'Voz_Salem', SoundRequest, 'sound_request')

        self.cheese_client = CheeseClient(self,Cheese, 'cheese',cui_service=self.voz_salem)    


        self.publisher = self.create_publisher(String, 'control_log', 10)



    def print_log(self, log_msg=''):
        msg = String()
        msg.data = f"SAlEM: {log_msg}"
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def go_salem(self):
        '''
        Aqui se establecerá el flujo de control principal de
        la rutina de ejecución de salem.
        '''

        self.voz_salem.build_request('n', 'Hola soy salem')
        self.voz_salem.send_request()

        self.voz_salem.build_request(
            'n', 'Preparate para posar, necesitamos tomarte unas fotos')
        self.voz_salem.send_request()

        self.cheese_client.send_goal('inicia')
            
    
def main(args=None):
    rclpy.init(args=args)

    salem_brain = ControlNode()
    executor = MultiThreadedExecutor()
    # Esto deberia permitir que salem ejecute peticiones asincronas
    executor.add_node(salem_brain)
    salem_brain.go_salem()
    rclpy.spin(salem_brain)

    salem_brain.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()