import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import threading
import time
import math
from flask import Flask, jsonify
from werkzeug.serving import make_server
from thymiodirect import Connection, Thymio
import sys
import traceback
from typing import List, Optional

# =============================================================================
# CONFIGURAÇÃO
# =============================================================================
# Constantes físicas do Thymio
SPEED_COEFF = 0.03027    # Coeficiente de conversão de velocidade (unidade Thymio/s para m/s)
ROBOT_WIDTH = 0.0935     # Eixo entre rodas (m)

# Constantes de Navegação
CRUISE_SPEED = 50        # Velocidade alvo do Thymio (unidades)
PROX_THRESHOLD = 2500    # Limiar para detetar obstáculo IR
LIDAR_SAFE_DIST = 0.40   # 40cm de segurança

# Configuração de Conexão
THYMIO_CONNECT_TIMEOUT = 3  # Tempo limite para a conexão (segundos)
THYMIO_CONNECT_RETRIES = 3  # Tentativas

app = Flask(__name__)
robot_node: Optional['ThymioBrain'] = None # Tipagem para acesso global

def euler_to_quaternion(roll, pitch, yaw) -> List[float]:
    """Converte ângulos de Euler (roll, pitch, yaw) para Quaternion."""
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    return [float(qx), float(qy), float(qz), float(qw)]

class ThymioBrain(Node):
    def __init__(self):
        super().__init__('thymio_driver')
        self.get_logger().info(">>> INICIANDO THYMIO BRAIN")
        
        self.th: Optional[Thymio] = None
        self.id: Optional[int] = None
        self.connected: bool = False
        
        # Tentativas de conexão com o Thymio
        for attempt in range(THYMIO_CONNECT_RETRIES):
            try:
                self.get_logger().info(f">>> TENTATIVA {attempt + 1}/{THYMIO_CONNECT_RETRIES} DE CONEXÃO")
                self.connect_thymio()
                if self.connected:
                    break
            except Exception as e:
                self.get_logger().error(f">>> FALHA NA TENTATIVA {attempt + 1}: {e}")
                time.sleep(1)
        
        if not self.connected:
            self.get_logger().warn(">>> NÃO FOI POSSÍVEL CONECTAR AO THYMIO. CONTINUANDO MODO OFFLINE.")
            
        # Inicialização do ROS
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        # Variáveis de Odometria
        self.x = 0.0
        self.y = 0.0
        self.th_yaw = 0.0
        self.last_time = self.get_clock().now()
        
        self.latest_scan: Optional[LaserScan] = None
        self.is_exploring: bool = False 
        
        # Timer de loop principal (50ms)
        self.create_timer(0.05, self.update_loop)
        self.get_logger().info(">>> THYMIO BRAIN INICIADO COM SUCESSO")

    def connect_thymio(self):
        """
        Conecta ao Thymio usando thread/join para impor o THYMIO_CONNECT_TIMEOUT,
        contornando a falta do argumento 'timeout' na biblioteca.
        """
        self.get_logger().info(">>> A CONECTAR AO THYMIO...")

        port = None
        try:
            # 1. Localizar a porta (serial ou network)
            port = Connection.serial_default_port()
            if not port:
                self.get_logger().warn(">>> NENHUMA PORTA SERIAL ENCONTRADA.")
                self.connected = False
                return
            self.get_logger().info(f">>> PORTA ENCONTRADA: {port}")
        except Exception as e:
            self.get_logger().error(f">>> ERRO AO PROCURAR PORTA: {e}")
            self.connected = False
            return

        result = {'success': False, 'error': None, 'th': None, 'node_id': None}
        
        # Função para correr a conexão que bloqueia
        def _connect_thread():
            try:
                self.get_logger().info(f">>> [THREAD] CONECTANDO A {port}...")
                
                # 1. CRIAR O OBJETO THYMIO (SEM O ARGUMENTO 'timeout')
                th = Thymio(
                    serial_port=port,  
                    on_connect=lambda node_id: self.get_logger().info(f'>>> [THREAD] THYMIO ENCONTRADO: {node_id}')
                )
                
                # 2. CONECTAR (Esta chamada bloqueia)
                th.connect()
                
                # 3. VERIFICAR NÓS
                node_id = th.first_node()
                
                if node_id:
                    result['success'] = True
                    result['th'] = th
                    result['node_id'] = node_id
                else:
                    result['error'] = "first_node() retornou None"
            except Exception as e:
                result['error'] = str(e)
                self.get_logger().error(f">>> [THREAD] ERRO: {e}")

        # Iniciar a thread e impor o timeout com join()
        thread = threading.Thread(target=_connect_thread, daemon=False)
        thread.start()
        
        # BLOQUEAR o thread principal até que a thread de conexão termine ou o timeout seja atingido
        thread.join(timeout=THYMIO_CONNECT_TIMEOUT)
        
        # Pós-processamento do resultado
        if thread.is_alive():
            self.get_logger().error(f">>> TIMEOUT NA CONEXÃO (>{THYMIO_CONNECT_TIMEOUT}s). O servidor Aseba não respondeu.")
            self.connected = False
            return
        
        if result['success']:
            try:
                self.th = result['th']
                self.id = result['node_id']
                
                # Teste final
                _ = self.th[self.id]["motor.left.target"]
                self.th[self.id]["leds.top"] = [0, 32, 0]  # LED Verde
                
                self.get_logger().info(">>> THYMIO LIGADO COM SUCESSO")
                self.connected = True
            except Exception as e:
                self.get_logger().error(f">>> ERRO AO TESTAR CONEXÃO: {e}")
                self.connected = False
        else:
            self.get_logger().error(f">>> FALHA NA CONEXÃO: {result['error']}")
            self.connected = False


    def scan_cb(self, msg: LaserScan):
        """Callback para receber dados do LIDAR."""
        self.latest_scan = msg

    def update_loop(self):
        """Loop principal de odometria e navegação (roda a 20 Hz)."""
        current_time = self.get_clock().now()
        
        # Sai do loop se não estiver conectado ou se o ID for nulo
        if not self.connected or self.th is None or self.id is None:
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = current_time

        try:
            # --- 1. ODOMETRIA ---
            # Nota: Usamos motor.left.target/motor.right.target para odometria,
            # mas o correto seria usar motor.left.speed/motor.right.speed (se a sua versão o suportar)
            motor_left = self.th[self.id]["motor.left.target"]
            motor_right = self.th[self.id]["motor.right.target"]
            prox = self.th[self.id]["prox.horizontal"]

            # Converter para signed (16-bit)
            if motor_left > 32767: motor_left -= 65536
            if motor_right > 32767: motor_right -= 65536

            # Cálculo de distância percorrida
            dl = motor_left * SPEED_COEFF * dt
            dr = motor_right * SPEED_COEFF * dt

            # Cálculo de alteração angular e linear
            delta_th = (dr - dl) / ROBOT_WIDTH
            ds = (dr + dl) / 2.0

            # Atualização da Pose
            mid_theta = self.th_yaw + (delta_th / 2.0)
            
            delta_x = ds * math.cos(mid_theta)
            delta_y = ds * math.sin(mid_theta)
            
            self.x += delta_x
            self.y += delta_y
            self.th_yaw += delta_th
            
            self.publish_tf(current_time, self.x, self.y, self.th_yaw)
            
            # --- 2. LÓGICA DE NAVEGAÇÃO ---
            tgt_l = 0
            tgt_r = 0

            if self.is_exploring:
                self.th[self.id]["leds.top"] = [0, 0, 32]  # LED Azul

                # Lógica de desvio com IR
                if max(prox) > PROX_THRESHOLD:
                    self.get_logger().info(f"[NAV] Obstaculo IR! Valor: {max(prox)}")
                    
                    obst_left = prox[0] + prox[1] + (prox[2] / 2)
                    obst_right = prox[4] + prox[3] + (prox[2] / 2)
                    
                    if obst_left > obst_right:
                        tgt_l = 100
                        tgt_r = -100  # Vira direita
                    else:
                        tgt_l = -100
                        tgt_r = 100    # Vira esquerda
                
                # Lógica de desvio com LIDAR
                elif self.latest_scan:
                    ranges = np.array(self.latest_scan.ranges)
                    ranges[ranges == 0] = 10.0
                    
                    # Setor frontal
                    front_idx = int(len(ranges) / 2)
                    front = np.min(ranges[front_idx-5:front_idx+5])
                    
                    if front < LIDAR_SAFE_DIST:
                        self.get_logger().info(f"[NAV] Obstaculo LIDAR! Dist: {front:.2f}m")
                        tgt_l = 0
                        tgt_r = 0
                    else:
                        tgt_l = CRUISE_SPEED
                        tgt_r = CRUISE_SPEED
                else:
                    # Sem obstáculos
                    tgt_l = CRUISE_SPEED
                    tgt_r = CRUISE_SPEED

            else:
                # Parado
                self.th[self.id]["leds.top"] = [0, 32, 0]  # LED Verde
                tgt_l = 0
                tgt_r = 0

            # Enviar para motores
            self.th[self.id]["motor.left.target"] = int(tgt_l)
            self.th[self.id]["motor.right.target"] = int(tgt_r)

        except Exception as e:
            self.get_logger().error(f">>> ERRO NO LOOP DE LEITURA/CONTROLO: {e}")
            traceback.print_exc()
            self.get_logger().warn(">>> DESCONECTANDO POR ERRO NO LOOP.")
            self.connected = False
            self.th = None


    def publish_tf(self, current_time: rclpy.time.Time, x: float, y: float, yaw: float):
        """Publica a transformação tf (odom -> base_link) e Odometria."""
        q = euler_to_quaternion(0, 0, yaw)
        
        # 1. Publicar TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publicar Odometria
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)

# =============================================================================
# FLASK API
# =============================================================================
@app.route('/start', methods=['GET'])
def start_exp():
    if robot_node and robot_node.connected:
        robot_node.is_exploring = True
        robot_node.get_logger().info(">>> RECEBI START")
        return jsonify({"status": "Started exploration"})
    return jsonify({"error": "Thymio não conectado ou Node não inicializado"}), 500

@app.route('/stop', methods=['GET'])
def stop_exp():
    if robot_node:
        robot_node.is_exploring = False
        if robot_node.connected and robot_node.th and robot_node.id:
            try:
                # Garante que os motores param
                robot_node.th[robot_node.id]["motor.left.target"] = 0
                robot_node.th[robot_node.id]["motor.right.target"] = 0
            except Exception:
                pass # Ignorar erro de desconexão ao tentar parar
        
        robot_node.get_logger().info(">>> RECEBI STOP")
        return jsonify({"status": "Stopped exploration"})
    return jsonify({"error": "Node não inicializado"}), 500

@app.route('/status', methods=['GET'])
def status():
    if robot_node:
        return jsonify({
            "status": "running",
            "thymio_connected": robot_node.connected,
            "exploring": robot_node.is_exploring,
            "position": {"x": round(robot_node.x, 3), "y": round(robot_node.y, 3), "theta": round(robot_node.th_yaw, 3)}
        })
    return jsonify({"status": "offline"}), 500

# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    """Main entry point com threading adequado para ROS2 e Flask."""
    global robot_node
    
    try:
        print(">>> INICIALIZANDO ROS2...")
        rclpy.init(args=args)
        
        print(">>> CRIANDO THYMIO BRAIN NODE...")
        robot_node = ThymioBrain()
        
        print(">>> INICIANDO ROS2 SPIN THREAD...")
        # Usa threading.Thread para rodar o ROS2 spin em background
        ros_thread = threading.Thread(target=rclpy.spin, args=(robot_node,), daemon=True)
        ros_thread.start()
        
        print(">>> INICIANDO FLASK SERVER NA PORTA 5000...")
        # Inicia o servidor Flask noutra thread
        server = make_server('0.0.0.0', 5000, app, threaded=True)
        flask_thread = threading.Thread(target=server.serve_forever, daemon=False)
        flask_thread.start()
        
        print(">>> SISTEMA PRONTO - AGUARDANDO COMANDOS")
        
        # Mantém a thread principal viva para evitar que os daemons morram
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n>>> ENCERRANDO...")
            
        finally:
            server.shutdown()
            robot_node.destroy_node()
            rclpy.shutdown()
            
    except Exception as e:
        print(f">>> ERRO FATAL: {e}")
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()