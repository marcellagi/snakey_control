import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np
from collections import namedtuple
from dynamixel_sdk_custom_interfaces.srv import GetPosition


class MinimalPublisher(Node):

    def __init__(self, param_1, param_2):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(SetPosition, '/set_position', 10) #cria um publicador
        timer_period = param_1  # período do temporizador em segundos
        self.sig = param_2  # sinal de entrada que será enviado aos atuadores
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.subscription = self.create_subscription( Joy, '/joy', self.joy_callback, 10)
        self.i = 0 
        self.motors = [0.0 for _ in range(6)] # inicializa a mensagem com 6 motores
        self.limite = range(1,3001) # limite de posição dos motores
        self.msg = SetPosition() #cria uma mensagem do tipo SetPosition
        self.joyA = 0 #armazena o valor do joystick
        self.controller = namedtuple('controller', 'left_H left_V right_H right_V bt_RB')
        self.client = self.create_client(GetPosition, 'get_position')
        self.zero_p =[0 for _ in range(6)]
        for i in range(6):
          self.zero_p[i] = self.zero_position(i+1)
        
    def zero_position(self, id):

      timeout_sec = 10.0
      if not self.client.wait_for_service(timeout_sec):
        raise RuntimeError('Service not available')
      req = GetPosition.Request()
      req.id = id
      future = self.client.call_async(req)
      rclpy.spin_until_future_complete(self, future)
      res = future.result()

      # print(f"Result: {res.position}")
      return int(res.position)
      # rclpy.shutdown()

    def pub_callback(self): 
      self.msg = SetPosition() #cria uma mensagem do tipo SetPosition

      if self.controller.bt_RB == 0:
        print("aleatorio-pub")
       
        for j in range(self.sig.shape[0]):
            self.msg.id.append(j+1) 
            self.msg.position.append(self.sig[j][self.i]+self.zero_p[j]) 
            self.get_logger().info('Motor %d: %s' %
                                    (j+1, self.sig[j][self.i]+self.zero_p[j]))         
        self.i += 1 #incrementa o contador de tempo 
        if (self.i >= self.sig.shape[1]):
            self.i = 0
      else:
        for j in range(self.sig.shape[0]): 
                  self.msg.id.append(j+1)
                  self.msg.position.append(int(self.motors[j])+self.zero_p[j])
                  print(int(self.motors[j]))
        
        self.controller_manager() 
        print("joy-pub")
        
      # send the message for the dynamixel_sdk
      self.publisher_.publish(self.msg)      

    def joy_callback(self, data): #retorno do temporizador, executado a cada intervalo de tempo
        self.controller.left_H = float(data.axes[0])
        self.controller.left_V = float(data.axes[1])
        self.controller.right_H = float(data.axes[3])
        self.controller.right_V = float(data.axes[4])
        self.controller.bt_RB = data.buttons[5]
        self.controller.bt_start = data.buttons[7]
    
    def map_range(self, inp, in_min, in_max, out_min, out_max):
      out = (inp - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
      if out > out_max:
        out = out_max
      elif out < out_min:   
        out = out_min         
      return out

    def controller_manager(self):
        if self.controller:
            controller = self.controller
            self.step = 100.0

            i1 = 0.625*controller.left_H + 0.353553*controller.left_V
            i2 = 0.625*controller.left_H + 0.625*controller.left_V
            i3 = 0.625*controller.left_H + 0.353553*controller.left_V
            
            i4 = controller.right_H - 0.3333*controller.right_V
            i5 = -controller.right_H - 0.3333*controller.right_V
            i6 = 0.6667*controller.right_V
           
            self.motors[0] = self.map_range(i1, 0, 1, 0, 2500) 
            self.motors[1] = self.map_range(i2, 0, 1, 0, 2500)
            self.motors[2] = self.map_range(i3, 0, 1, 0, 2500)
            self.motors[3] = self.map_range(i4, 0, 1, 0, 2500)
            self.motors[4] = self.map_range(i5, 0, 1, 0, 2500)
            self.motors[5] = self.map_range(i6, 0, 1, 0, 2500)

            # if controller.left_H > 0.1:
            #   self.motors[1] = self.map_range(i2, 0.3, 1, 0, 2500)
            #   self.motors[2] = self.map_range(i3, 0.3, 1, 0, 2500)
           
                
            # elif controller.left_H < 0.1:
            #   self.motors[0] = self.map_range(i1, 0.3, 1, 0, 2500) 
            #   self.motors[1] = self.map_range(i2, 0.3, 1, 0, 2500)
             
            
            # if controller.left_V > 0.1:
            #   self.motors[0] = self.map_range(i1, 0.3, 1, 0, 2500)
            #   self.motors[2] = self.map_range(i3, 0.3, 1, 0, 2500)
              
            
            # elif controller.left_V < 0.1:
            #   self.motors[2] = self.map_range(i3, 0.3, 1, 0, 2500)

            # if controller.right_H > 0.1:
            #   self.motors[4] = self.map_range(i5, 0.3, 1, 0, 2500)
            #   self.motors[5] = self.map_range(i6, 0.3, 1, 0, 2500)

            # elif controller.right_H < 0.1:
            #   self.motors[3] = self.map_range(i4, 0.3, 1, 0, 2500)
            #   self.motors[5] = self.map_range(i6, 0.3, 1, 0, 2500)

            # if controller.right_V > 0.1:
            #   self.motors[5] = self.map_range(i6, 0.3, 1, 0, 2500)

            # elif controller.right_V < 0.1:
            #   self.motors[3] = self.map_range(i4, 0.3, 1, 0, 2500)
            #   self.motors[4] = self.map_range(i5, 0.3, 1, 0, 2500)

            

def APRBS(a_range, b_range, nstep): #gera um sinal de entrada aleatório que será usado como entrada      
    #define a faixa de amplitude do sinal
    a = np.random.rand(nstep) * (a_range[1]-a_range[0]) + a_range[0] 
    # define faixa de frequência do sinal
    b = np.random.rand(nstep) * (b_range[1]-b_range[0]) + b_range[0]
    b = np.round(b)
    b = b.astype(int)

    b[0] = 0 

    for i in range(1, np.size(b)): 
        b[i] = b[i-1]+b[i]

    # sinal randomico
    i = 0
    random_signal = np.zeros(nstep)
    while b[i] < np.size(random_signal):
        k = b[i]
        random_signal[k:] = a[i]
        i = i+1

      # PRBS
    a = np.zeros(nstep)
    j = 0
    while j < nstep:
        a[j] = 5
        a[j+1] = -5
        j = j+2

    i = 0
    prbs = np.zeros(nstep)
    while b[i] < np.size(prbs):
        k = b[i]
        prbs[k:] = a[i]
        i = i+1
    return random_signal


def multiple_aprbs(a_range, b_range, nstep, ninput, type='float', factor=1.0):
    u = np.zeros([ninput, nstep])
    for i in range(ninput):
        u[i, :] = APRBS(a_range, b_range, nstep)*factor
    if type == 'int':
        u = u.astype('int32')
    return u


def main(args=None):
    rclpy.init(args=args)
    node_time = 0.25
    ninput = 6
    nstep = 3600
    a_range = [0, 2.0]  # the motor range are 0 to 4000 - factor = 1000
    b_range = [0, 15]  # rate signal range [MIN TIME, MAX TIME]
    # form the random signal to motors
    u = multiple_aprbs(a_range, b_range, nstep,
                       ninput, type='int', factor=1000.0)

    minimal_publisher = MinimalPublisher(node_time, u)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
