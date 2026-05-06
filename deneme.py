import carla
import time
import cv2
import numpy as np

# --- HAFIZA MERKEZİ ---
son_sol_serit = None
son_sag_serit = None

# --- 1. KOORDİNAT HESAPLAMA ---
def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0] 
    y2 = int(y1 * (1/2)) 
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

# --- 2. ŞERİTLERİ ORTALAMA VE HAFIZADA TUTMA ---
def average_slope_intercept(image, lines):
    global son_sol_serit, son_sag_serit
    left_fit = []
    right_fit = []
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 == x2: continue 
            
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            
            if slope < -0.1: 
                left_fit.append((slope, intercept))
            elif slope > 0.1: 
                right_fit.append((slope, intercept))
    
    if len(left_fit) > 0:
        son_sol_serit = np.average(left_fit, axis=0)
    if len(right_fit) > 0:
        son_sag_serit = np.average(right_fit, axis=0)
        
    lines_to_return = []
    if son_sol_serit is not None:
        lines_to_return.append(make_coordinates(image, son_sol_serit))
    if son_sag_serit is not None:
        lines_to_return.append(make_coordinates(image, son_sag_serit))
        
    return np.array(lines_to_return)

# --- 3. BEYİN VE GÖRÜNTÜ KONTROL MERKEZİ ---
def process_image(image, vehicle):
    i = np.array(image.raw_data)
    i2 = i.reshape((360, 640, 4))
    i3 = i2[:, :, :3] 
    
    gray_image = cv2.cvtColor(i3, cv2.COLOR_BGR2GRAY)
    canny_edges = cv2.Canny(gray_image, 50, 150)
    
    height, width = canny_edges.shape
    polygons = np.array([ [(0, height), (width, height), (320, 180)] ])
    mask = np.zeros_like(canny_edges)
    cv2.fillPoly(mask, polygons, 255)
    cropped_edges = cv2.bitwise_and(canny_edges, mask)
    
    lines = cv2.HoughLinesP(cropped_edges, 2, np.pi/180, 50, np.array([]), minLineLength=20, maxLineGap=200)
    averaged_lines = average_slope_intercept(i3, lines)
    
    line_image = np.zeros_like(i3)
    steering_angle = 0.0 
    throttle = 0.40      

    line_count = len(averaged_lines) if averaged_lines is not None else 0

    if line_count == 2:
        left_x1 = averaged_lines[0][0]
        right_x1 = averaged_lines[1][0]
        lane_center = int((left_x1 + right_x1) / 2)
        car_center = 320 
        
        error = lane_center - car_center
        steering_angle = error * 0.005 
        
        cv2.line(line_image, (averaged_lines[0][0], averaged_lines[0][1]), (averaged_lines[0][2], averaged_lines[0][3]), (0, 255, 0), 10)
        cv2.line(line_image, (averaged_lines[1][0], averaged_lines[1][1]), (averaged_lines[1][2], averaged_lines[1][3]), (0, 255, 0), 10)
        cv2.circle(line_image, (lane_center, 300), 10, (255, 0, 0), -1) 
        cv2.circle(line_image, (car_center, 360), 10, (0, 0, 255), -1)  

    elif line_count == 1:
        cv2.line(line_image, (averaged_lines[0][0], averaged_lines[0][1]), (averaged_lines[0][2], averaged_lines[0][3]), (0, 255, 255), 10)
        steering_angle = 0.0 
        throttle = 0.30      
    else:
        steering_angle = 0.0
        throttle = 0.20

    steering_angle = max(-1.0, min(1.0, steering_angle))

    control = carla.VehicleControl()
    control.throttle = throttle
    control.steer = steering_angle
    vehicle.apply_control(control)
            
    combo_image = cv2.addWeighted(i3, 0.8, line_image, 1, 1)
    
    # SADECE TEK PENCERE AÇIYORUZ (Yapay Zeka Gözü)
    cv2.imshow("Autonex Zekasi (Algoritma)", combo_image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        import os
        os._exit(0)

# --- 4. SİSTEMİ AYAĞA KALDIRMA ---
def main():
    print("Matrix'e bağlanılıyor...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0) 
    
    print("Town04 (Otoban) yükleniyor...")
    world = client.load_world('Town04')
    blueprint_library = world.get_blueprint_library()

    vehicle_bp = blueprint_library.filter('model3')[0]
    
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[45] if len(spawn_points) > 45 else spawn_points[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '360')
    camera_bp.set_attribute('fov', '90')

    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    camera.listen(lambda image: process_image(image, vehicle))

    print("\n>>> AUTONEX OTONOM MOD DEVREDE! <<<")
    print(">>> CARLA oyun ekranından arabayı tepe kamerasıyla izleyebilirsin! <<<\n")

    try:
        while True:
            # --- GTA TARZI TEPE KAMERASI (Sürekli Takip) ---
            spectator = world.get_spectator()
            transform = vehicle.get_transform()
            
            # Kamerayı arabanın 8 metre arkasına ve 5 metre yukarısına sabitle
            ileri_vektoru = transform.get_forward_vector()
            kamera_konumu = transform.location - carla.Location(x=ileri_vektoru.x * 8, y=ileri_vektoru.y * 8) + carla.Location(z=5.0)
            
            # Kameranın bakış açısını al ve burnunu hafifçe (-15 derece) aşağı eğ
            kamera_acisi = transform.rotation
            kamera_acisi.pitch -= 15.0 
            
            spectator.set_transform(carla.Transform(kamera_konumu, kamera_acisi))
            
            time.sleep(0.05) # Kameranın daha akıcı takip etmesi için süreyi kısalttık
    except KeyboardInterrupt:
        pass
    finally:
        camera.destroy()
        vehicle.destroy()
        cv2.destroyAllWindows()
        print("Sistem güvenle kapatıldı.")

if __name__ == '__main__':
    main()