#from FourPointsMethod  import*
import socket
import time
import struct
import math
import numpy as np

# Setze die IP-Adresse des Roboters
#robot_ip = '172.28.178.77'

## Kommunikation mit UR herstellen
def send_urscript(script, host):
    port = 30002  # UR secondary client port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    s.send(script.encode('utf-8'))
    s.close()

## Erstellen einer movej Bewegung für UR  ##Angaben in grad/S
def generate_urscript_movej(x, y, z, rx, ry, rz, a=0.1, v=0.1):
    urscript = f"""
def move_to_position():
    movej(p[{x}, {y}, {z}, {rx}, {ry}, {rz}], a={a}, v={v})
    textmsg("Movement complete!")
end

move_to_position()
"""
    return urscript

## Erstellen einer movel Bewegung für UR ##Angaben in m
def generate_urscript_movel(x, y, z, rx, ry, rz, a=0.1, v=0.1):
    urscript = f"""
def move_to_position():
    movel(p[{x}, {y}, {z}, {rx}, {ry}, {rz}], a={a}, v={v})
    textmsg("Movement complete!")
end

move_to_position()
"""
    return urscript

## Erstellen einer MoveP-Bewegung für UR
def generate_urscript_movep(x, y, z, rx, ry, rz, a=0.1, v=0.1, r=0.0):
    urscript = f"""
def move_to_position():
    movep(p[{x}, {y}, {z}, {rx}, {ry}, {rz}], a={a}, v={v}, r={r})
    textmsg("Movement complete!")
end

move_to_position()
"""
    return urscript

def rotate_wrist3(robot_ip, delta_angle_deg, a=0.1, v=0.1, threshold=0.001):
    joint_angles = get_joint_angles(robot_ip)
    base, shoulder, elbow, wrist1, wrist2, wrist3 = joint_angles

    # Berechnung des neuen Zielwinkels
    delta_angle_rad = math.radians(delta_angle_deg)
    new_wrist3 = wrist3 + delta_angle_rad

    urscript = f"""
def rotate_wrist3():
    movej([{base}, {shoulder}, {elbow}, {wrist1}, {wrist2}, {new_wrist3}], a={a}, v={v})
    textmsg("Wrist3 rotation complete")
end

rotate_wrist3()
"""
    send_urscript(urscript, robot_ip)

    # Warten, bis die Rotation abgeschlossen ist
    while True:
        current_joint_angles = get_joint_angles(robot_ip)
        current_wrist3 = current_joint_angles[-1]
        if abs(current_wrist3 - new_wrist3) < threshold:
            break
        time.sleep(0.1)


def wait_for_position(robot_ip, target_position, threshold=0.005):
    while True:
        current_position = get_current_position(robot_ip)
        current_yz = current_position[1:3]
        if has_reached_position(current_yz, target_position, threshold):
            print("wait_for_position: mission complete!\n")
            break
        time.sleep(0.1)  # Kurze Pause zwischen den Abfragen

def get_current_position(robot_ip):
    port = 30003  # Real-time data port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((robot_ip, port))
    data = s.recv(1108)
    s.close()
    
    # Extract the position data from the received packet correctly
    unpacked_data = struct.unpack('!6d', data[444:492])  # 6 double values starting at byte 444
    x, y, z, rx, ry, rz = unpacked_data
    
    return [x, y, z, rx, ry, rz]

def get_joint_angles(robot_ip):
    
    ## Liefert Winkel der einzelnen Achsen (Base, shoulder usw.) in Rad

    PORT = 30003  # UR robots typically use port 30003 for primary interface
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((robot_ip, PORT))
    data = s.recv(2048)
    s.close()
    
    # Extract the position data from the received packet correctly
    joint_angles = struct.unpack('!6d', data[252:300])
    base, shoulder, elbow, wrist1, wrist2, wrist3 = joint_angles

    return [base, shoulder, elbow, wrist1, wrist2, wrist3]

def has_reached_position(current_pos, target_pos, threshold=0.005):
    # Compare only the x, y, z coordinates
    for current, target in zip(current_pos[:3], target_pos[:3]):
        if abs(current - target) > threshold:
            return False
    return True

def move_to_main_position(robot_ip):
    main_position = [-0.460, -0.03210, 0.45522, 1.188, 4.364, -1.000]
    script = generate_urscript_movej(*main_position)
    send_urscript(script, robot_ip)

     # Wait until the robot reaches the position
    while True:
        # current_position = get_joint_angles(robot_ip)
        current_position = get_current_position(robot_ip)
        if has_reached_position(current_position, main_position):
            break
        time.sleep(0.1)   

def move_to_prestart_position(robot_ip):
    start_position = [-0.850, -0.3150, 0.32309, 0.065, 4.703, 0.145]
    script = generate_urscript_movel(*start_position)
    send_urscript(script, robot_ip)

     # Wait until the robot reaches the position
    while True:
        # current_position = get_joint_angles(robot_ip)
        current_position = get_current_position(robot_ip)
        if has_reached_position(current_position, start_position):
            break
        time.sleep(0.1)


def move_to_start_position(robot_ip):
    start_position = [-0.900, -0.3150, 0.32309, 0.065, 4.703, 0.145]
    script = generate_urscript_movel(*start_position)
    send_urscript(script, robot_ip)

     # Wait until the robot reaches the position
    while True:
        # current_position = get_joint_angles(robot_ip)
        current_position = get_current_position(robot_ip)
        if has_reached_position(current_position, start_position):
            break
        time.sleep(0.1)

def move_to_end_position(robot_ip):
    # Aktuelle Position abrufen
    current_position = get_current_position(robot_ip)
    
    # X-Wert um 0.1 m erhöhen
    new_x = current_position[0] + 0.2
    end_position = [new_x, current_position[1], current_position[2],
                    current_position[3], current_position[4], current_position[5]]
    
    script = generate_urscript_movel(*end_position)
    send_urscript(script, robot_ip)

    # wait for position
    while True:
        current_position = get_current_position(robot_ip)
        if has_reached_position(current_position, end_position):
            break
        time.sleep(0.1)


def send_batch_movements(robot_ip, points, a=0.05, v=0.05, r=0.002):
    urscript = "def batch_movements():\n"
    for point in points:
        urscript += f"  movep(p[{point[0]}, {point[1]}, {point[2]}, {point[3]}, {point[4]}, {point[5]}], a={0.05}, v={0.05}, r={r})\n"
    urscript += "  textmsg(\"Batch movements complete\")\n"
    urscript += "end\n\nbatch_movements()"
    send_urscript(urscript, robot_ip)


def process_filtered(filtered_list, robot_ip, transform_matrix, a=0.1, v=0.1, rotation_deg=30, threshold=0.005):
    batch_points = []  # Temporäre Liste für Batch-Punkte

    # Auslesen Winkel der 6-Achse (Wrist3)
    current_joint_angles = get_joint_angles(robot_ip)
    current_wrist3 = current_joint_angles[-1]  

    # Auslesen der kartesischen Koordinaten für rz
    current_positions = get_current_position(robot_ip)
    current_rx = current_positions[3]
    current_ry = current_positions[4]
    current_rz = current_positions[5]

    while filtered_list:
        action, y, x = filtered_list.pop(0)

        if action not in [4, 5]:
            # Umrechnung in Roboterkoordinaten mit 4-Punkt-Methode
            pixel_point = np.array([x, y, 1])
            # print(f"Processing Pixel Point: {pixel_point}")

            robot_point = transform_matrix @ pixel_point
            robot_point = np.asarray(robot_point).flatten()  # In 1D-Array umwandeln
            # print(f"Transformed Robot Point (raw): {robot_point}")

            # Sicherstellen, dass die Koordinaten normalisiert sind (kann wahrscheinlich raus)
            if len(robot_point) == 4 and robot_point[3] != 0:
                # Normalisierung: Teilen durch die vierte Koordinate
                robot_point = robot_point[:3] / robot_point[3]
            else:
                raise ValueError(f"Unexpected transformed point: {robot_point}")

            # Umrechnung von mm in m
            x_robot, y_robot, z_robot = robot_point / 1000

            # print(f'x_robot={x_robot},y_robot={y_robot}z_robot={z_robot}')
            
            x_robot = -0.900
            batch_points.append([x_robot, y_robot, z_robot, current_rx, current_ry, current_rz])
        else:
            if batch_points:
                # Bevor die Liste gelöscht wird, speichere die letzten y, z Koordinaten
                last_position = batch_points[-1][1:3]
                
                # Batch an den Roboter senden
                send_batch_movements(robot_ip, batch_points, a, v)
                batch_points = []  # Punkte zurücksetzen

                # Warte, bis die Bewegungen abgeschlossen sind, nur mit y und z
                wait_for_position(robot_ip, last_position, threshold)

            # Rotation durchführen
            delta_angle = -rotation_deg if action == 5 else rotation_deg
            rotate_wrist3(robot_ip, delta_angle, a, v)

            # Aktualisieren der Rotationsvektoren
            current_joint_angles = get_joint_angles(robot_ip)
            current_wrist3 = current_joint_angles[-1]  

            # Auslesen der kartesischen Koordinaten für rz
            current_positions = get_current_position(robot_ip)
            current_rx = current_positions[3]
            current_ry = current_positions[4]
            current_rz = current_positions[5]
            time.sleep(0.1)

    # Letzte Bewegungen nach Durchlaufen der Liste senden (falls vorhanden)
    if batch_points:
        send_batch_movements(robot_ip, batch_points, a, v)
        # Warte, bis die letzten Bewegungen abgeschlossen sind
        last_position = batch_points[-1][1:3]  # Nur y und z für den Abgleich
        wait_for_position(robot_ip, last_position, threshold)

#not used
## überarbeiten filtered_list 
def load_filtered_list(filename):
    filtered_list = []
    with open(filename, 'r') as file:
        for line in file:
            # Entferne Leerzeichen und Zeilenumbrüche und teile die Werte durch das Komma
            values = line.strip().split(',')
            # Wandelt die Werte in Integer um und speichert sie als Tuple in der Liste
            filtered_list.append((int(values[0]), int(values[1]), int(values[2])))
    return filtered_list



   # Calculation of the transformation matrix
    #four_points_method = FourPointsMethod("points.json")
    #transform_matrix = four_points_method.transform_pix_mm
    
    move_to_main_position(robot_ip)
"""     move_to_prestart_position(robot_ip)
    move_to_start_position(robot_ip)

    filtered_list = load_filtered_list('filtered_list.txt')

    # Drehwinkel für RZ in Grad
    rz_rotation_degrees = 45

    # Aufruf der Funktion mit Transformationsmatrix
    process_filtered(filtered_list, robot_ip, transform_matrix, rotation_deg=rz_rotation_degrees)

    move_to_end_position(robot_ip)
    move_to_main_position(robot_ip) """
