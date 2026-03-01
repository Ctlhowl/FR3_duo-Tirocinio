import subprocess
import time
import random
import math
import os
import signal

# --- PARAMETRI DELLA SPAWN OGGETTO ---
BASKET_CENTER_X = -0.04
BASKET_CENTER_Y = -0.50

R_MIN = 0.25
R_MAX = 0.7
Z_ROTATION_MIN = -math.pi
Z_ROTATION_MAX = math.pi

# --- PARAMETRI DELLO SCAFFALE ---
END_X_MIN = -0.4
END_X_MAX = 0.5

NUM_RUNS = 10

def genera_start_pose():
    """Genera coordinate [x, y, yaw] uniformi all'interno del recinto."""
    u = random.uniform(0.0, 1.0)
    r = math.sqrt(u * (R_MAX**2 - R_MIN**2) + R_MIN**2)
    theta = random.uniform(-math.pi, math.pi)
    
    x = BASKET_CENTER_X + r * math.cos(theta)
    y = BASKET_CENTER_Y + r * math.sin(theta)
    yaw = random.uniform(-math.pi, math.pi)
    
    return [round(x, 2), round(y, 2), round(yaw, 2)]

def genera_end_pose():
    """Genera coordinate [x, y] casuali per lo scaffale."""
    end_x = random.uniform(END_X_MIN, END_X_MAX)
    end_y = random.choice([0.0, 1.0, 2.0])
    
    return [round(end_x, 2), round(end_y, 2)]

# --- INIZIO BENCHMARK ---
successi = 0
fallimenti = 0
tempi_esecuzione = []

print(f"Inizio benchmark: {NUM_RUNS} esecuzioni con generazione casuale (Pick & Place completi).")

for i in range(NUM_RUNS):
    # Genera pose casuali
    start_pose = genera_start_pose()
    end_pose = genera_end_pose()
    
    start_pose_str = f"[{start_pose[0]}, {start_pose[1]}, {start_pose[2]}]"
    end_pose_str = f"[{end_pose[0]}, {end_pose[1]}]"
    
    print(f"\n--- Run {i+1}/{NUM_RUNS} ---")
    print(f" -> Start Pose (Recinto) : {start_pose_str}")
    print(f" -> End Pose   (Scaffale): {end_pose_str}")
    
    # Costruisci il comando dinamicamente
    comando_launch = [
        "ros2", "launch", "task_constructor", "mtc.launch.py", 
        "load_gripper:=true", "use_sim:=true", 
        f"start_pose:={start_pose_str}", 
        f"end_pose:={end_pose_str}"
    ]
    
    start_time = time.time()
    process = subprocess.Popen(
        comando_launch, 
        stdout=subprocess.PIPE, 
        stderr=subprocess.STDOUT, 
        text=True,
        preexec_fn=os.setsid 
    )
    
    esito_run = "FALLITO" 
    
    for line in process.stdout:
        if "Pianificazione del task fallita" in line or "Codice Errore" in line:
            esito_run = "FALLITO"
            print(" -> [!] Errore rilevato durante la pianificazione o l'esecuzione.")
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            break
        elif "Cubo rimosso con successo" in line:
            esito_run = "SUCCESSO"
            successi += 1
            print(" -> [✓] Task completato con successo.")
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            break

    process.wait() 
    end_time = time.time()
    
    if esito_run == "SUCCESSO":
        tempi_esecuzione.append(end_time - start_time)
    else:
        fallimenti += 1
        
    time.sleep(4) # Pausa di sicurezza per il simulatore

# --- STATISTICHE FINALI ---
print("\n" + "="*40)
print("========= RISULTATI BENCHMARK =========")
print("="*40)
print(f"Esecuzioni totali  : {NUM_RUNS}")
print(f"Successi           : {successi} ({(successi/NUM_RUNS)*100:.1f}%)")
print(f"Fallimenti         : {fallimenti} ({(fallimenti/NUM_RUNS)*100:.1f}%)")
if tempi_esecuzione:
    tempo_medio = sum(tempi_esecuzione) / len(tempi_esecuzione)
    tempo_minimo = min(tempi_esecuzione)
    tempo_massimo = max(tempi_esecuzione)
    print("-" * 40)
    print(f"Tempo medio per run: {tempo_medio:.2f} secondi")
    print(f"Tempo minimo       : {tempo_minimo:.2f} secondi")
    print(f"Tempo massimo      : {tempo_massimo:.2f} secondi")
print("="*40)