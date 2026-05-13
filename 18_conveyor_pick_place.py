"""
DERS 5: PYBULLET İLE CONVEYOR BELT PICK & PLACE (GELİŞTİRİLMİŞ)
================================================================

PyBullet Avantajları:
✓ %100 Python - Lua script yok!
✓ Güçlü fizik simülasyonu
✓ Gerçek robot'a kolay geçiş
✓ ROS entegrasyonu
✓ Machine Learning için ideal

Kurulum:
pip install pybullet numpy

GELİŞTİRMELER:
✓ Franka Panda robot (gripper dahil!)
✓ Conveyor belt robota çarpmayacak şekilde yerleştirildi
✓ Daha iyi pozisyonlama
✓ Gelişmiş gripper kontrolü

"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import math

class PyBulletConveyorSystem:
    """
    PyBullet ile Conveyor Belt Pick-Place Sistemi
    Franka Panda Robot ile
    """
    
    def __init__(self, gui=True):
        """
        Args:
            gui: True ise görsel arayüz açılır
        """
        
        # PyBullet başlat
        if gui:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # GUI'yi basitleştir
            print("✅ PyBullet GUI modu başlatıldı")
        else:
            self.physics_client = p.connect(p.DIRECT)
            print("✅ PyBullet DIRECT modu başlatıldı")
        
        # Ek data yollarını ekle
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Yerçekimi ayarla
        p.setGravity(0, 0, -9.81)
        
        # Zaman adımı
        p.setTimeStep(1./240.)
        
        # Objeler
        self.robot_id = None
        self.conveyor_id = None
        self.objects = []
        self.gripper_constraint = None
        
        # Conveyor parametreleri (Robot'un YANINDA, üstünde değil!)
        self.conveyor_speed = 0.08  # m/s
        self.conveyor_length = 1.5
        self.conveyor_width = 0.4
        self.conveyor_position = [0.5, 0.5, 0.05]  # Robot'un yanında
        
        # Robot konumu
        self.robot_base_position = [0, 0, 0]
        
        # İstatistikler
        self.stats = {
            'spawned': 0,
            'detected': 0,
            'picked': 0,
            'dropped': 0,
            'failed': 0
        }
        
        # Sahneyi kur
        self._setup_scene()
        
        print("✅ PyBullet Conveyor Sistemi Hazır!")
        print(f"   Robot: Franka Panda @ {self.robot_base_position}")
        print(f"   Conveyor: @ {self.conveyor_position}")
    
    # ==================== SAHNE KURULUMU ====================
    
    def _setup_scene(self):
        """Sahneyi kur: Zemin, robot, conveyor belt"""
        
        print("\n🏗️  Sahne kuruluyor...")
        
        # 1. Zemin (plane)
        self.plane_id = p.loadURDF("plane.urdf")
        
        # 2. Robot yükle (Franka Panda - gripper dahil!)
        self._load_franka_panda()
        
        # 3. Conveyor Belt oluştur (Robot'un YANINDA)
        self._create_conveyor_belt()
        
        # 4. Bırakma kutusu (drop zone)
        self._create_drop_zone()
        
        # 5. Kamera ayarla
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=45,
            cameraPitch=-35,
            cameraTargetPosition=[0.3, 0.3, 0.3]
        )
        
        print("✅ Sahne kurulumu tamamlandı!")
    
    def _load_franka_panda(self):
        """Franka Panda robot yükle (gripper dahil!)"""
        
        print("   → Franka Panda yükleniyor...")
        
        try:
            # Franka Panda robot
            self.robot_id = p.loadURDF(
                "franka_panda/panda.urdf",
                basePosition=self.robot_base_position,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION
            )
            
            # Joint bilgilerini al
            self.num_joints = p.getNumJoints(self.robot_id)
            
            # Franka Panda joint yapısı:
            # 0-6: Arm joints (7-DOF)
            # 7-8: Gripper joints (finger joints)
            self.arm_joints = list(range(7))  # 7-DOF arm
            self.gripper_joints = [9, 10]     # Gripper fingers
            
            # End-effector link (panda_hand)
            self.end_effector_link = 11  # panda_hand link
            
            print(f"   ✅ Franka Panda yüklendi (ID: {self.robot_id})")
            print(f"      Toplam joint: {self.num_joints}")
            print(f"      Arm joints: {self.arm_joints}")
            print(f"      Gripper joints: {self.gripper_joints}")
            
            # Joint limitlerini al
            self._get_joint_limits()
            
            # Home pozisyonuna git
            home_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
            self.move_to_joint_positions(home_position)
            
            # Gripper'ı başlangıçta aç
            self.gripper_open()
            
        except Exception as e:
            print(f"❌ Franka Panda yüklenemedi: {e}")
            print("   Lütfen PyBullet'in güncel olduğundan emin olun!")
            raise
    
    def _create_conveyor_belt(self):
        """Conveyor belt oluştur (Robot'un yanında!)"""
        
        print("   → Conveyor belt oluşturuluyor...")
        
        # Görsel şekil
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.conveyor_length/2, self.conveyor_width/2, 0.05],
            rgbaColor=[0.3, 0.3, 0.3, 1]
        )
        
        # Çarpışma şekli
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.conveyor_length/2, self.conveyor_width/2, 0.05]
        )
        
        # Conveyor belt oluştur
        self.conveyor_id = p.createMultiBody(
            baseMass=0,  # Sabit
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=self.conveyor_position
        )
        
        print(f"   ✅ Conveyor belt oluşturuldu @ {self.conveyor_position}")
        
        # Yön göstergesi (ok) - opsiyonel
        arrow_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.05, 0.02, 0.001],
            rgbaColor=[1, 1, 0, 1]
        )
        
        arrow_pos = [
            self.conveyor_position[0] - self.conveyor_length/2 + 0.1,
            self.conveyor_position[1],
            self.conveyor_position[2] + 0.11
        ]
        
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=arrow_visual,
            basePosition=arrow_pos
        )
    
    def _create_drop_zone(self):
        """Bırakma kutusu oluştur"""
        
        print("   → Bırakma kutusu oluşturuluyor...")
        
        # Kutu görsel
        box_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.15, 0.15, 0.1],
            rgbaColor=[0.2, 0.8, 0.2, 0.3]
        )
        
        # Kutu pozisyonu (Robot'un diğer tarafı)
        drop_position = [0, -0.5, 0.1]
        
        self.drop_zone_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=box_visual,
            basePosition=drop_position
        )
        
        self.drop_position = [drop_position[0], drop_position[1], drop_position[2] + 0.15]
        
        print(f"   ✅ Bırakma kutusu @ {drop_position}")
    
    def _get_joint_limits(self):
        """Joint limitlerini al"""
        self.joint_limits = []
        
        for joint_idx in self.arm_joints:
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            self.joint_limits.append((lower_limit, upper_limit))
    
    # ==================== ROBOT KONTROLÜ ====================
    
    def move_to_joint_positions(self, target_positions, max_velocity=0.5):
        """
        Joint pozisyonlarına git (smooth hareket)
        
        Args:
            target_positions: Hedef joint açıları (radyan)
            max_velocity: Maksimum hız
        """
        
        # Her joint için pozisyon kontrolü
        for i, joint_idx in enumerate(self.arm_joints):
            if i < len(target_positions):
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_positions[i],
                    maxVelocity=max_velocity,
                    force=200
                )
        
        # Simülasyonu çalıştır (hareketi tamamla)
        # Hareketin bitmesini bekle
        for _ in range(480):  # ~1 saniye
            p.stepSimulation()
            time.sleep(1./240.)
            
            # Hedefe yaklaşıp yaklaşmadığını kontrol et
            current_positions = [p.getJointState(self.robot_id, j)[0] for j in self.arm_joints[:len(target_positions)]]
            diff = np.abs(np.array(current_positions) - np.array(target_positions))
            if np.all(diff < 0.01):  # Yeterince yakın
                break
    
    def get_end_effector_pose(self):
        """
        End-effector'ın pozisyon ve yönlendirmesini al
        
        Returns:
            (position, orientation) tuple
        """
        
        link_state = p.getLinkState(
            self.robot_id,
            self.end_effector_link,
            computeForwardKinematics=True
        )
        
        position = link_state[4]  # World position
        orientation = link_state[5]  # World orientation (quaternion)
        
        return position, orientation
    
    def inverse_kinematics(self, target_position, target_orientation=None):
        """
        Ters kinematik çözümü
        
        Args:
            target_position: [x, y, z]
            target_orientation: Quaternion (opsiyonel)
        
        Returns:
            Joint açıları
        """
        
        if target_orientation is None:
            # Varsayılan yönlendirme (aşağı bakan)
            target_orientation = p.getQuaternionFromEuler([math.pi, 0, 0])
        
        joint_poses = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.end_effector_link,
            targetPosition=target_position,
            targetOrientation=target_orientation,
            maxNumIterations=100,
            residualThreshold=0.001
        )
        
        return joint_poses[:len(self.arm_joints)]
    
    def move_to_position(self, target_position, target_orientation=None, velocity=0.5):
        """
        Cartesian pozisyona git (IK kullanarak)
        
        Args:
            target_position: [x, y, z]
            target_orientation: Quaternion (opsiyonel)
            velocity: Hareket hızı
        """
        
        # IK hesapla
        joint_positions = self.inverse_kinematics(target_position, target_orientation)
        
        # Joint pozisyonlarına git
        self.move_to_joint_positions(joint_positions, max_velocity=velocity)
    
    # ==================== GRIPPER KONTROLÜ ====================
    
    def gripper_open(self, width=0.04):
        """
        Gripper'ı aç
        
        Args:
            width: Açıklık miktarı (metre)
        """
        
        for joint_idx in self.gripper_joints:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=width,
                force=20
            )
        
        # Gripper'ın açılmasını bekle
        for _ in range(60):
            p.stepSimulation()
            time.sleep(1./240.)
    
    def gripper_close(self, force=40):
        """
        Gripper'ı kapat
        
        Args:
            force: Kapama kuvveti
        """
        
        for joint_idx in self.gripper_joints:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.0,
                force=force
            )
        
        # Gripper'ın kapanmasını bekle
        for _ in range(60):
            p.stepSimulation()
            time.sleep(1./240.)
    
    def check_gripper_grasp(self):
        """
        Gripper'da nesne var mı kontrol et
        
        Returns:
            bool: Nesne yakalandı mı?
        """
        
        # Gripper joint'lerinin mevcut pozisyonlarını al
        finger_states = [p.getJointState(self.robot_id, j)[0] for j in self.gripper_joints]
        
        # Eğer gripper tam kapanmamışsa, bir şey tutuyordur
        avg_position = np.mean(finger_states)
        
        # Kapanma toleransı
        return avg_position > 0.005  # 5mm'den fazla açıksa nesne var demektir
    
    # ==================== NESNE YÖNETİMİ ====================
    
    def spawn_object(self, position=None, size=0.03):
        """
        Band üzerinde nesne oluştur
        
        Args:
            position: [x, y, z] - None ise otomatik
            size: Nesne boyutu
        
        Returns:
            object_id
        """
        
        if position is None:
            # Conveyor başlangıcında rastgele pozisyon
            conv_start_x = self.conveyor_position[0] - self.conveyor_length/2 + 0.15
            conv_y = self.conveyor_position[1] + np.random.uniform(-0.12, 0.12)
            conv_z = self.conveyor_position[2] + 0.15
            position = [conv_start_x, conv_y, conv_z]
        
        # Rastgele şekil seç
        shape_choice = np.random.choice(['box', 'cylinder', 'sphere'])
        
        if shape_choice == 'box':
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size, size, size],
                rgbaColor=[np.random.rand(), np.random.rand(), np.random.rand(), 1]
            )
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size, size, size]
            )
        elif shape_choice == 'cylinder':
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=size,
                length=size*2,
                rgbaColor=[np.random.rand(), np.random.rand(), np.random.rand(), 1]
            )
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=size,
                height=size*2
            )
        else:  # sphere
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=size,
                rgbaColor=[np.random.rand(), np.random.rand(), np.random.rand(), 1]
            )
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_SPHERE,
                radius=size
            )
        
        # Obje oluştur
        obj_id = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        
        self.objects.append({
            'id': obj_id,
            'spawn_time': time.time(),
            'picked': False,
            'shape': shape_choice
        })
        
        self.stats['spawned'] += 1
        
        print(f"📦 {shape_choice.upper()} oluşturuldu (ID: {obj_id}) @ [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]")
        
        return obj_id
    
    def update_conveyor(self, dt=1./240.):
        """
        Conveyor belt hareketini simüle et
        Band üzerindeki nesneleri hareket ettir
        
        Args:
            dt: Zaman adımı (saniye)
        """
        
        # Band üzerindeki tüm nesneleri bul ve hareket ettir
        for obj_data in self.objects[:]:  # Copy ile iterate et (silme için güvenli)
            obj_id = obj_data['id']
            
            # Nesne hala var mı kontrol et
            try:
                position, orientation = p.getBasePositionAndOrientation(obj_id)
                
                # X ekseninde ileri hareket (band yönü)
                new_x = position[0] + self.conveyor_speed * dt
                
                # Band sonuna ulaştıysa sil
                conv_end_x = self.conveyor_position[0] + self.conveyor_width/2
                if new_x > conv_end_x + 0.2:
                    p.removeBody(obj_id)
                    self.objects.remove(obj_data)
                    continue
                
                # Yeni pozisyon
                new_position = [new_x, position[1], position[2]]
                
                # Pozisyonu güncelle (sadece yakalanmamış objeler için)
                if not obj_data['picked']:
                    p.resetBasePositionAndOrientation(obj_id, new_position, orientation)
                
            except:
                # Nesne silinmişse listeden çıkar
                if obj_data in self.objects:
                    self.objects.remove(obj_data)
    
    def detect_objects_in_zone(self, zone_x_min=None, zone_x_max=None):
        """
        Belirli bir bölgedeki nesneleri algıla (X EKSENİNDE)

        Args:
            zone_x_min: Bölge X minimum
            zone_x_max: Bölge X maksimum
        
        Returns:
            Liste: Algılanan nesneler
        """
        
        if zone_x_min is None:
            # Robot'un erişebileceği alan (X ekseninde)
            zone_x_min = self.conveyor_position[0] - 0.15 # Bu değerleri robotun erişimine göre ayarlayabilirsiniz
        
        if zone_x_max is None:
            zone_x_max = self.conveyor_position[0] + 0.15 # Bu değerleri robotun erişimine göre ayarlayabilirsiniz
        
        detected = []
        
        for obj_data in self.objects:
            if obj_data['picked']:
                continue
            
            obj_id = obj_data['id']
            
            try:
                position, _ = p.getBasePositionAndOrientation(obj_id)
                
                # Bölge kontrolü (X ekseninde)
                if zone_x_min <= position[0] <= zone_x_max:
                    # Y ekseninde conveyor üzerinde mi?
                    conv_y_min = self.conveyor_position[1] - self.conveyor_width/2
                    conv_y_max = self.conveyor_position[1] + self.conveyor_width/2
                    
                    if conv_y_min <= position[1] <= conv_y_max:
                        detected.append({
                            'id': obj_id,
                            'position': position,
                            'data': obj_data
                        })
                        self.stats['detected'] += 1
            except:
                pass
        
        return detected
    
    # ==================== PICK & PLACE ====================
    
    def pick_object(self, target_position, approach_height=0.35):
        """
        Nesneyi yakala
        
        Args:
            target_position: Hedef pozisyon [x, y, z]
            approach_height: Yaklaşma yüksekliği
        
        Returns:
            bool: Başarılı mı?
        """
        
        print(f"\n🎯 Yakalama başlıyor: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")
        
        # 1. Gripper'ı aç
        self.gripper_open()
        print("   → Gripper açıldı")
        
        # 2. Yaklaşma pozisyonuna git (yüksekte)
        approach_pos = list(target_position)
        approach_pos[2] = approach_height
        
        print("   → Yaklaşma pozisyonuna gidiliyor...")
        self.move_to_position(approach_pos, velocity=0.8)
        
        # 3. Nesneye doğru in
        print("   → Nesneye doğru iniliyor...")
        grasp_pos = list(target_position)
        grasp_pos[2] = target_position[2]  # Biraz üstten yakala
        
        self.move_to_position(grasp_pos, velocity=0.3)
        
        # 4. Gripper'ı kapat
        print("   → Gripper kapatılıyor...")
        self.gripper_close()
        
        # 5. Kontrol et
        has_object = self.check_gripper_grasp()
        
        if has_object:
            print("   ✅ Nesne yakalandı!")
            self.stats['picked'] += 1
            
            # 6. Yukarı kaldır
            lift_pos = list(grasp_pos)
            lift_pos[2] = approach_height
            self.move_to_position(lift_pos, velocity=0.5)
            
            return True
        else:
            print("   ❌ Nesne yakalanamadı!")
            self.stats['failed'] += 1
            
            # Gripper'ı aç ve geri çekil
            self.gripper_open()
            self.move_to_position(approach_pos, velocity=0.5)
            
            return False
    
    def place_object(self):
        """
        Nesneyi bırakma kutusuna bırak
        """
        
        print(f"\n📦 Nesne bırakılıyor @ [{self.drop_position[0]:.3f}, {self.drop_position[1]:.3f}, {self.drop_position[2]:.3f}]")
        
        # 1. Bırakma pozisyonuna git
        self.move_to_position(self.drop_position, velocity=0.8)
        
        # 2. Gripper'ı aç
        self.gripper_open()
        
        self.stats['dropped'] += 1
        print("   ✅ Nesne bırakıldı!")
        
        time.sleep(0.3)
    
    def go_home(self):
        """Home pozisyonuna dön"""
        print("\n🏠 Home pozisyonuna dönülüyor...")
        home_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        self.move_to_joint_positions(home_position, max_velocity=0.8)
    
    # ==================== ANA DÖNGÜ ====================
    
    def run_single_cycle(self):
        """Tek bir pick-place döngüsü"""
        
        print("\n" + "="*60)
        print("🔄 TEK DÖNGÜ")
        print("="*60)
        
        # Gözlem pozisyonuna git
        observe_position = [0, -0.3, 0, -1.5, 0, 1.2, 0.785]
        self.move_to_joint_positions(observe_position)
        
        # Biraz bekle (nesne yaklaşsın)
        print("⏳ Nesne bekleniyor...")
        for _ in range(60):
            self.update_conveyor()
            p.stepSimulation()
            time.sleep(1./240.)
        
        # Nesneleri algıla
        detected = self.detect_objects_in_zone()
        
        if not detected:
            print("❌ Yakalama bölgesinde nesne yok!")
            return False
        
        # En yakın nesneyi seç
        target = detected[0]
        print(f"🎯 Hedef: ID={target['id']} @ [{target['position'][0]:.3f}, {target['position'][1]:.3f}, {target['position'][2]:.3f}]")
        
        # Yakala
        success = self.pick_object(target['position'])
        
        if success:
            # Yakalanan nesneyi işaretle
            target['data']['picked'] = True
            
            # Bırak
            self.place_object()
            
            # Home'a dön
            self.go_home()
            
            return True
        else:
            # Home'a dön
            self.go_home()
            return False
    
    def run_continuous(self, duration=60, spawn_interval=5):
        """
        Sürekli pick-place döngüsü
        
        Args:
            duration: Toplam süre (saniye)
            spawn_interval: Nesne oluşturma aralığı (saniye)
        """
        
        print("\n" + "="*70)
        print("🤖 FRANKA PANDA CONVEYOR PICK-PLACE SİSTEMİ")
        print("="*70)
        print(f"Süre: {duration} saniye")
        print(f"Nesne oluşturma aralığı: {spawn_interval} saniye")
        print(f"Band pozisyonu: {self.conveyor_position}")
        print(f"Robot pozisyonu: {self.robot_base_position}")
        print()
        
        start_time = time.time()
        last_spawn_time = start_time
        last_pick_time = start_time
        
        # Gözlem pozisyonuna git
        observe_position = [0, -0.3, 0, -1.5, 0, 1.2, 0.785]
        self.move_to_joint_positions(observe_position)
        
        # Ana döngü
        step_count = 0
        while time.time() - start_time < duration:
            
            # Conveyor'ı güncelle
            self.update_conveyor()
            
            # Yeni nesne oluştur
            if time.time() - last_spawn_time > spawn_interval:
                self.spawn_object()
                last_spawn_time = time.time()
            
            # Pick-place işlemi (her 10 saniyede bir)
            if time.time() - last_pick_time > 10:
                detected = self.detect_objects_in_zone()
                
                if detected:
                    target = detected[0]
                    print(f"\n🎯 Hedef seçildi: ID={target['id']}")
                    
                    success = self.pick_object(target['position'])
                    
                    if success:
                        target['data']['picked'] = True
                        self.place_object()
                    
                    self.go_home()
                    
                    # Gözlem pozisyonuna geri dön
                    self.move_to_joint_positions(observe_position)
                
                last_pick_time = time.time()
            
            # Simülasyonu ilerlet
            p.stepSimulation()
            time.sleep(1./240.)
            step_count += 1
            
            # Her 240 adımda bir (1 saniye) durum mesajı
            if step_count % 240 == 0:
                elapsed = time.time() - start_time
                remaining = duration - elapsed
                print(f"⏱️  Geçen: {elapsed:.1f}s | Kalan: {remaining:.1f}s | Toplam nesne: {len(self.objects)}")
        
        # İstatistikleri göster
        self.print_statistics()
        
        print("\n" + "="*70)
        print("✅ SİMÜLASYON TAMAMLANDI!")
        print("="*70)
    
    def print_statistics(self):
        """İstatistikleri yazdır"""
        
        print("\n" + "="*60)
        print("📊 İSTATİSTİKLER")
        print("="*60)
        print(f"   Oluşturulan nesneler: {self.stats['spawned']}")
        print(f"   Algılanan nesneler: {self.stats['detected']}")
        print(f"   Yakalanan nesneler: {self.stats['picked']}")
        print(f"   Bırakılan nesneler: {self.stats['dropped']}")
        print(f"   Başarısız denemeler: {self.stats['failed']}")
        
        if self.stats['picked'] + self.stats['failed'] > 0:
            success_rate = self.stats['picked'] / (self.stats['picked'] + self.stats['failed']) * 100
            print(f"   Başarı oranı: {success_rate:.1f}%")
        print("="*60)
    
    def cleanup(self):
        """Simülasyonu kapat"""
        p.disconnect()
        print("\n✅ PyBullet bağlantısı kapatıldı")


# ==================== DEMO VE TEST ====================

def demo_basic():
    """Temel demo - Manuel test"""
    
    print("\n🎮 TEMEL DEMO - Manuel Nesne Yakalama\n")
    
    # Sistemi başlat
    system = PyBulletConveyorSystem(gui=True)
    
    print("\n📦 3 nesne oluşturuluyor...")
    
    # Birkaç nesne oluştur (conveyor üzerinde farklı pozisyonlarda)
    for i in range(3):
        x_pos = system.conveyor_position[0] - system.conveyor_length/2 + 0.3 + i * 0.3
        y_pos = system.conveyor_position[1]
        z_pos = system.conveyor_position[2] + 0.15
        system.spawn_object([x_pos, y_pos, z_pos])
    
    print("\n⏳ Nesnelerin durması bekleniyor...")
    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./240.)
    
    # Her nesneyi sırayla yakala
    for cycle in range(3):
        print(f"\n{'='*60}")
        print(f"DÖNGÜ {cycle + 1}/3")
        print(f"{'='*60}")
        
        success = system.run_single_cycle()
        
        if not success:
            print("⚠️  Bu döngü atlandı, devam ediliyor...")
        
        time.sleep(1)
    
    # İstatistikler
    system.print_statistics()
    
    print("\n✅ Demo tamamlandı!")
    print("Pencereyi kapatmak için ENTER'a basın...")
    input()
    
    system.cleanup()


def demo_continuous():
    """Sürekli döngü demo - Otomatik sistem"""
    
    print("\n🎮 SÜREKLI DÖNGÜ DEMO - Otomatik Conveyor Sistemi\n")
    
    # Sistemi başlat
    system = PyBulletConveyorSystem(gui=True)
    
    try:
        # 60 saniye çalıştır
        system.run_continuous(duration=60, spawn_interval=6)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Kullanıcı tarafından durduruldu (Ctrl+C)")
        system.print_statistics()
    
    finally:
        print("\nPencereyi kapatmak için ENTER'a basın...")
        input()
        system.cleanup()


def demo_interactive():
    """İnteraktif demo - Kullanıcı kontrolü"""
    
    print("\n🎮 İNTERAKTİF DEMO\n")
    
    system = PyBulletConveyorSystem(gui=True)
    
    print("\nKomutlar:")
    print("  s - Nesne oluştur (spawn)")
    print("  p - Tek pick-place döngüsü")
    print("  h - Home pozisyonuna git")
    print("  o - Gripper aç")
    print("  c - Gripper kapat")
    print("  q - Çıkış")
    print()
    
    try:
        while True:
            cmd = input("Komut girin: ").lower().strip()
            
            if cmd == 's':
                system.spawn_object()
            
            elif cmd == 'p':
                # Conveyor'ı biraz çalıştır
                print("⏳ Band çalışıyor...")
                for _ in range(120):
                    system.update_conveyor()
                    p.stepSimulation()
                    time.sleep(1./240.)
                
                system.run_single_cycle()
            
            elif cmd == 'h':
                system.go_home()
            
            elif cmd == 'o':
                system.gripper_open()
            
            elif cmd == 'c':
                system.gripper_close()
            
            elif cmd == 'q':
                break
            
            else:
                print("❌ Geçersiz komut!")
            
            # Simülasyonu biraz ilerlet
            for _ in range(10):
                system.update_conveyor()
                p.stepSimulation()
                time.sleep(1./240.)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Kullanıcı tarafından durduruldu")
    
    finally:
        system.print_statistics()
        system.cleanup()


def main():
    """Ana program"""
    
    print("\n" + "="*70)
    print("🤖 FRANKA PANDA CONVEYOR PICK-PLACE SİSTEMİ")
    print("="*70)
    print("\nGELİŞTİRMELER:")
    print("  ✓ Franka Panda robot (gerçek gripper!)")
    print("  ✓ Conveyor belt robot'un yanında (çarpışma yok!)")
    print("  ✓ Daha iyi IK ve hareket kontrolü")
    print("  ✓ Farklı şekiller (box, cylinder, sphere)")
    print("  ✓ Gelişmiş gripper kontrolü")
    print()
    print("DEMO SEÇENEKLERİ:")
    print("1. Temel Demo (3 nesne manuel test)")
    print("2. Sürekli Döngü (60 saniye otomatik)")
    print("3. İnteraktif Mod (elle kontrol)")
    print("4. Çıkış")
    print("="*70)
    
    choice = input("\nSeçiminiz (1-4): ")
    
    if choice == '1':
        demo_basic()
    elif choice == '2':
        demo_continuous()
    elif choice == '3':
        demo_interactive()
    elif choice == '4':
        print("👋 Görüşürüz!")
    else:
        print("❌ Geçersiz seçim!")


if __name__ == '__main__':
    main()