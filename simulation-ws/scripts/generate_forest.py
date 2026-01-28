import random
import math
import os

# --- MODELOS ---
MODELS = []
MODELS += [f"bush_{i}" for i in range(1, 4)]    
MODELS += [f"dtree_{i}" for i in range(1, 4)]   
MODELS += [f"tree_{i}" for i in range(1, 6)]    

FILENAME = "floresta_esparsa_.world"
TARGET_DIR = os.path.join("..", "src", "gazebo_resources", "worlds")

# --- CONFIGURAÇÕES ---
NUM_OBJECTS = 400  
AREA_SIZE = 120.0  
# Aumentei para 10 metros para garantir que o LiDAR não pegue árvore colada no início
SAFE_ZONE_RADIUS = 10.0
# Distancia entre os modelos para nao ter um "dentro" do outro
MIN_DIST_MODELS = 1.0
MAX_DIST_MODELS = 3.0 

header = """<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include><uri>model://sun</uri></include>
    
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>250 250</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>250 250</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grass</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>
"""

footer = """
  </world>
</sdf>
"""

def generate_world():
    
    if not os.path.exists(TARGET_DIR):
        final_path = FILENAME
    else:
        final_path = os.path.join(TARGET_DIR, FILENAME)
    
    with open(final_path, "w") as f:
        f.write(header)
        count = 0
        utilizedPoints = []
        while count < NUM_OBJECTS:
            x = random.uniform(-AREA_SIZE/2, AREA_SIZE/2)
            y = random.uniform(-AREA_SIZE/2, AREA_SIZE/2)
            #Verificação para nao ter modelos coincidentes 
            dist_beetwen_models = round(random.uniform(MIN_DIST_MODELS,MAX_DIST_MODELS),1)
            too_close = False
            for px,py in utilizedPoints:
              dist = math.sqrt((x-px)**2 + (y-py)**2)
              if dist < dist_beetwen_models:
                too_close = True
                break
            if too_close:
                continue
              
            # Verificação de distância euclidiana real d = sqrt(x^2 + y^2)
            distance_from_center = math.sqrt(x**2 + y**2)
            if distance_from_center < SAFE_ZONE_RADIUS:
                continue # Pula se estiver dentro do círculo central de 10m

            model_name = random.choice(MODELS)
            yaw = random.uniform(0, 2 * math.pi)
            
            obj_xml = f"""
    <include>
      <uri>model://{model_name}</uri>
      <name>obj_{count}_{model_name}</name>
      <pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.2f}</pose>
    </include>"""
            f.write(obj_xml)
            utilizedPoints.append((x,y))
            count += 1
            
        f.write(footer)
    
    print(f"Mundo gerado com sucesso! Zona central de {SAFE_ZONE_RADIUS}m protegida.")

if __name__ == "__main__":
    generate_world()