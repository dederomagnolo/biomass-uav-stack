#!/usr/bin/env python3
import rospy
import time
from mrs_msgs.srv import Vec4
from mrs_msgs.srv import Vec4Request

# Configurações
UAV_NAME = "uav1"
SERVICE_NAME = f"/{UAV_NAME}/control_manager/goto"
ALTURA_VOO = 2.5  # Altura boa para ver troncos e copas

# Lista de Waypoints [x, y, z, yaw]
# O Yaw 0 aponta para o X positivo.
traj = [
    # 1. Calibração inicial (Movimento em 8 para IMU)
    [5.0,  0.0,  ALTURA_VOO, 0.0],
    [5.0,  5.0,  ALTURA_VOO, 1.57],
    [0.0,  5.0,  ALTURA_VOO, 3.14],
    [0.0,  0.0,  ALTURA_VOO, 0.0],
    
    # 2. Exploração da Floresta (Grid / Zig-Zag)
    # Ajuste esses valores conforme o tamanho da sua floresta (AREA_SIZE)
    [15.0, 0.0,  ALTURA_VOO, 0.0],   # Avança
    [15.0, 15.0, ALTURA_VOO, 1.57],  # Sobe lateral
    [-15.0, 15.0, ALTURA_VOO, 3.14], # Cruza o mapa
    [-15.0, 0.0,  ALTURA_VOO, -1.57], # Desce lateral
    
    # 3. Retorno para Loop Closure (MUITO IMPORTANTE)
    [0.0, 0.0, ALTURA_VOO, 0.0]
]

def fly_mission():
    rospy.init_node('mission_commander', anonymous=True)
    
    # Espera o serviço de controle estar disponível
    print("Aguardando serviço de controle...")
    rospy.wait_for_service(SERVICE_NAME)
    
    goto_service = rospy.ServiceProxy(SERVICE_NAME, Vec4)
    
    print(f"Iniciando missão com {len(traj)} waypoints...")
    
    for i, p in enumerate(traj):
        if rospy.is_shutdown():
            break
            
        print(f"Indo para Waypoint {i+1}: X={p[0]}, Y={p[1]}")
        
        # Cria a mensagem
        req = Vec4Request()
        req.goal = p
        
        # Envia o comando
        try:
            resp = goto_service(req)
            if not resp.success:
                print(f"ERRO ao enviar waypoint {i+1}: {resp.message}")
        except rospy.ServiceException as e:
            print(f"Falha na chamada do serviço: {e}")

        # TEMPO DE ESPERA SIMPLES
        # Calculamos um tempo estimado para chegar (distância / velocidade média)
        # Assumindo velocidade de ~2m/s + margem
        time.sleep(8) 

    print("Missão concluída! Pousando...")
    # Opcional: chamar serviço de pouso aqui se quiser
    # rospy.ServiceProxy(f"/{UAV_NAME}/uav_manager/land", Trigger)()

if __name__ == '__main__':
    try:
        fly_mission()
    except rospy.ROSInterruptException:
        pass