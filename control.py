import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np


class MinimalPublisher(Node):

    def __init__(self, param_1, param_2):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(SetPosition, '/set_position', 10) 
        timer_period = param_1  # período do temporizador em segundos
        self.sig = param_2  # sinal de entrada que será enviado aos atuadores
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscriptions = self.create_subscription( Joy, '/joy', self.callback, 10)
        self.i = 0
        self.motors = [SetPosition() for _ in range(6)] # inicializa a mensagem com 6 motores
        self.limite = 3000
        self.msg = SetPosition() #cria uma mensagem do tipo SetPosition
        for i in range(1, 7):   
          self.motors[i].id = [i]
          self.motors[i].position = SetPosition() # posição do motor 1
        

    def timer_callback(self): #retorno do temporizador, executado a cada intervalo de tempo

        if self.i <= 30:
            for j in range(self.sig.shape[0]):  # menor que 30 segundos todos os motores em 10
                self.msg.id.append(j+1)
                self.msg.position.append(10)
        else:
            for j in range(self.sig.shape[0]):  # maior que 30 segundos todos os motores na posição do sinal 
                self.msg.id.append(j+1)
                self.msg.position.append(self.sig[j][self.i-31]+10)
                self.get_logger().info('Motor %d: %s' %
                                       (j+1, self.sig[j][self.i-31]))
        self.i += 1 #incrementa o contador de tempo 
        if (self.i >= self.sig.shape[1]+10): #se o contador de tempo for maior que o tamanho do sinal, reinicia o contador
            self.i = 0
        self.publisher_.publish(self.msg)
        # self.get_logger().info(msg)

    def callback(self, data):

        A, B, C, D, E = data.axes[0], data.axes[1], data.axes[3], data.axes[4], data.buttons[0]

        if E:
            if A > 0:
            # ligar motor que vai para esq
                self.motors[2] += 10 * A if self.motors[2] < self.limite else 0.0
                self.motors[6] += 10 * A if self.motors[6] < self.limite else 0.0
                
            elif A < 0:
            # ligar motor que vai para dir
                self.motors[2] -= 10 * A if self.motors[2] < self.limite else 0.0
                self.motors[4] += 10 * A if self.motors[4] < self.limite else 0.0
        
            if B > 0:
            # ligar motor que vai para frente
                self.motors[2] += 10 * B if self.motors[2] < self.limite else 0.0
            elif B < 0:
            # ligar motor que vai para trás
                self.motors[2] -= 10 * B if self.motors[2] < self.limite else 0.0
            if C > 0:
            # ligar motor que vai para esq
                self.motors[3] += 10 * B if self.motors[3] < self.limite else 0.0
                self.motors[5] += 10 * B if self.motors[5] < self.limite else 0.0
            elif C < 0:
            # ligar motor que vai para dir
                self.motors[3] -= 10 * B if self.motors[3] < self.limite else 0.0
                self.motors[5] -= 10 * B if self.motors[5] < self.limite else 0.0
            if D > 0:
            # ligar motor que vai para esq
                self.motors[1] -= 10 * B if self.motors[1] < self.limite else 0.0
            elif D < 0:
            # ligar motor que vai para dir
                self.motors[1] -= 10 * B if self.motors[1] < self.limite else 0.0
            # formatacao dos dados
            for i in range(1, 7):   
              self.publisher_.publish(self.motors[i])
        else:
            self.publisher_.publish(self.msg)

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
    node_time = 1.0
    ninput = 6
    nstep = 3600
    a_range = [0, 2.5]  # the motor range are 0 to 4000 - factor = 1000
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
