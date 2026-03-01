import subprocess
import time
import random
import math

# --- PARAMETRI DELLA SPAWN OGGETTO ---
R_MIN = 0.35
R_MAX = 0.70 
Z_ROTATION_MIN = -math.pi
Z_ROTATION_MAX = math.pi

# --- PARAMETRI DELLO SCAFFALE ---
END_X_MIN = -0.5
END_X_MAX = 0.5
END_Y_MIN = 0.0
END_Y_MAX = 2.0

NUM_RUNS = 20

def genera_start_pose():
    """Genera coordinate [x, y, yaw] uniformi all'interno del recinto."""
    u = random.uniform(0.0, 1.0)
    r = math.sqrt(u * (R_MAX**2 - R_MIN**2) + R_MIN**2)
    
    theta = random.uniform(-math.pi, math.pi)
    
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    yaw = random.uniform(Z_ROTATION_MIN, Z_ROTATION_MAX)
    
    return [round(x, 3), round(y, 3), round(yaw, 3)]

def genera_end_pose():
    """Genera coordinate [x, y] casuali per lo scaffale."""
    end_x = random.uniform(END_X_MIN, END_X_MAX)
    end_y = random.uniform(END_Y_MIN, END_Y_MAX)
    
    return [round(end_x, 3), round(end_y, 3)]

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
    process = subprocess.Popen(comando_launch, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    
    esito_run = "FALLITO" 
    
    for line in process.stdout:
        if "Pianificazione del task fallita" in line or "Codice Errore" in line:
            esito_run = "FALLITO"
            print(" -> [!] Errore rilevato durante la pianificazione o l'esecuzione.")
            process.terminate()
            break
        elif "Cubo rimosso con successo" in line:
            esito_run = "SUCCESSO"
            successi += 1
            print(" -> [✓] Task completato con successo.")
            process.terminate()
            break

    process.wait() 
    end_time = time.time()
    
    if esito_run == "SUCCESSO":
        tempi_esecuzione.append(end_time - start_time)
    else:
        fallimenti += 1
        
    time.sleep(2) # Pausa di sicurezza per il simulatore

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