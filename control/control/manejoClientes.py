import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup



class ActionClientManager():
    '''
    Clase que se encarga de dar un  manejo general 
    a los clientes que se conectan con action services 
    '''
    def __init__(self,node,action_type, action_name):
        # Esto es para evitar los dealocks por usar el mismo grupo para el manejo de callbacks
        group = ReentrantCallbackGroup()
        self.client = ActionClient(node, action_type, action_name,callback_group=group)
        self.node = node
        self.__goal_future = None
        self.__result_future = None

    def build_goal_msg(self,*args):
        '''
        Esta funcion debe de indicar como 
        se como generar un goal_msg 

        Ejemplo
        -------
        ```python
            goal_msg = action_type.Goal()
            goal_msg.order = args[0]
            return goal_msg
        ``` 
        '''
        raise NotImplementedError

    def result_callback(self, future):
        '''
        Esta función maneja lo que se debe hacer al cumplirse la promesa

        Ejemplo:
        -------
        ```python
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        ```
        '''
        raise NotImplementedError

    def feedback_callback(self, feedback_msg):
        '''
        Esta función se encarga del manejo del feedback, si no se implementa
        no realiza accion alguna

        Ejemplo
        -------
        ```python
        feedback = feedback_msg.feedback
        message = 'Received feedback: {0}'.format(feedback.progress)
        self.get_logger().info(message)
        ```
        '''
        pass

    def send_goal(self,*args):
        '''
        Manda una peticion al action service y asigna los callbacks de feedback
        y respuesta de meta a la promesa enviada.

        `WARNING:` denbe de implemetarse build_goal_msg para usarla sin errores
        '''
        goal_msg = self.build_goal_msg(*args)

        self.client.wait_for_server()

        self.__goal_future = self.client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self.__goal_future.add_done_callback(self.done_callback)   
       

    def done_callback(self, future):
        '''
        Esta funcion es llamada al recibir una respuesta a la meta 
        propuesta por el cliente
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return

        self.node.get_logger().info('Goal accepted :)')

        self.__result_future = goal_handle.get_result_async()
        self.__result_future.add_done_callback(self.result_callback)



class ClientAsync(Node):
    '''
    Clase que se encarga de abstraer los clientes de servicios de ros2
    Las peticiones se realizan de manera asincrona

    Para customizar la peticion, las clases que extiendan de esta deberian
    implementar alguna función que le de forma a la petición.
    '''

    def __init__(self, client_name, interface_type, service_name):
        super().__init__(client_name)
        #Esto es para evitar los dealocks por usar el mismo grupo para el manejo de callbacks
        group = ReentrantCallbackGroup()
        self.cli = self.create_client(interface_type,
                                      service_name, 
                                      callback_group=group)

        self.req = interface_type.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
