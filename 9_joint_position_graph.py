# GELİŞTİRİLMİŞ ROBOT KONTROL SİMÜLASYONU
# DERS 3+: Gelişmiş Eklem Durumu Okuma ve Kontrol

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import threading

class RobotSimulator:
    def __init__(self, gui=True):
        """Robot simülatörünü başlatır ve robotu yükler"""
        self.gui = gui
        self.simulation_running = False
        self.joint_states_history = {i: deque(maxlen=100) for i in range(7)}  # Son 100 adımı sakla
        self.time_history = deque(maxlen=100)

        # Simülasyonu başlat
        if self.gui:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # GUI'yi basitleştir
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)  # Adım adım simülasyon
        p.setTimeStep(1./240.)

        # Zemin ekle
        p.loadURDF("plane.urdf")

        # Robotu yükle
        self.robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot)

        # Eklem bilgilerini al
        self.joint_info = []
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            self.joint_info.append({
                'name': info[1].decode('utf-8'),
                'type': info[2],
                'lower_limit': info[8],
                'upper_limit': info[9],
                'max_force': info[10],
                'max_velocity': info[11]
            })

        # Eklem kontrol parametreleri
        self.joint_gains = [0.1] * self.num_joints
        self.joint_damping = [0.5] * self.num_joints

        print("✓ Simülasyon başlatıldı")
        print(f"✓ Robot yüklendi: {self.num_joints} eklem tespit edildi")

        # Başlangıç pozisyonunu sıfırla
        self.reset_robot()

    def reset_robot(self):
        """Robotu başlangıç pozisyonuna sıfırlar"""
        for i in range(self.num_joints):
            p.resetJointState(self.robot, i, 0.0)
        print("✓ Robot başlangıç pozisyonuna sıfırlandı")

    def get_joint_state(self, joint_index):
        """
        Bir eklemin gerçek durumunu okur

        Parametreler:
            joint_index: Eklem numarası

        Döndürür:
            position: Eklemin mevcut açısı (radyan)
            velocity: Eklemin dönüş hızı (radyan/saniye)
            reaction_forces: Reaksiyon kuvvetleri
            applied_torque: Uygulanan tork
        """
        try:
            state = p.getJointState(self.robot, joint_index)
            position, velocity, reaction_forces, applied_torque = state
            return position, velocity, reaction_forces, applied_torque
        except Exception as e:
            print(f"Hata: Eklem {joint_index} durumu okunamadı: {e}")
            return None, None, None, None

    def set_joint_position(self, joint_index, target_position, max_force=None):
        """
        Bir eklemi hedef pozisyona hareket ettirir

        Parametreler:
            joint_index: Eklem numarası
            target_position: Hedef açı (radyan)
            max_force: Maksimum uygulanan kuvvet (None ise otomatik)
        """
        if max_force is None:
            max_force = self.joint_info[joint_index]['max_force']

        p.setJointMotorControl2(
            bodyUniqueId=self.robot,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            force=max_force,
            positionGain=self.joint_gains[joint_index],
            velocityGain=self.joint_damping[joint_index]
        )

    def set_joint_positions(self, target_positions):
        """
        Birden fazla eklemi aynı anda hedef pozisyonlara hareket ettirir

        Parametreler:
            target_positions: Hedef açılar listesi (radyan)
        """
        for i, target in enumerate(target_positions):
            if i < self.num_joints:
                self.set_joint_position(i, target)

    def simulate(self, steps=1):
        """Simülasyonu belirtilen adım kadar ilerletir"""
        for _ in range(steps):
            p.stepSimulation()
            if self.gui:
                time.sleep(1./240.)

    def update_joint_history(self):
        """Eklem durumları geçmişini günceller"""
        current_time = time.time()
        self.time_history.append(current_time)

        for i in range(self.num_joints):
            pos, vel, _, _ = self.get_joint_state(i)
            self.joint_states_history[i].append((pos, vel))

    def print_joint_states(self, joint_indices=None):
        """
        Eklem durumlarını ekrana yazdırır

        Parametreler:
            joint_indices: Yazdırılacak eklem numaraları (None ise tüm eklemler)
        """
        if joint_indices is None:
            joint_indices = range(self.num_joints)

        print("\n" + "="*50)
        print("EKLEM DURUMLARI")
        print("="*50)

        for i in joint_indices:
            if i < self.num_joints:
                pos, vel, _, _ = self.get_joint_state(i)
                if pos is not None:
                    print(f"Eklem {i} ({self.joint_info[i]['name']}):")
                    print(f"  Açı: {pos:.3f} radyan = {math.degrees(pos):.1f} derece")
                    print(f"  Hız: {vel:.3f} rad/s")

    def get_end_effector_position(self):
        """Robotun son efektörünün pozisyonunu döndürür"""
        end_effector_link = self.num_joints - 1
        state = p.getLinkState(self.robot, end_effector_link)
        position, orientation = state[0], state[1]
        return position, orientation

    def plot_joint_positions(self, joint_indices=None, show_plot=True):
        """
        Eklem pozisyonlarını grafiğe döker

        Parametreler:
            joint_indices: Çizdirilecek eklem numaraları (None ise tüm eklemler)
            show_plot: Grafiği hemen göster (True) veya figürü döndür (False)
        """
        if joint_indices is None:
            joint_indices = range(self.num_joints)

        plt.figure(figsize=(10, 6))

        for i in joint_indices:
            if i < self.num_joints and len(self.joint_states_history[i]) > 0:
                positions = [state[0] for state in self.joint_states_history[i]]
                times = list(range(len(positions)))
                plt.plot(times, positions, label=f"Eklem {i} ({self.joint_info[i]['name']})")

        plt.xlabel("Zaman (adım)")
        plt.ylabel("Pozisyon (radyan)")
        plt.title("Eklem Pozisyonları")
        plt.legend()
        plt.grid(True)

        if show_plot:
            plt.show()
            return None
        else:
            return plt.gcf()

    def run_test_1(self):
        """TEST 1: Başlangıç pozisyonunu oku"""
        print("\n" + "="*50)
        print("TEST 1: Başlangıç Pozisyonu")
        print("="*50)

        self.print_joint_states(range(3))

        input("\nDevam etmek için ENTER'a basın...")

    def run_test_2(self):
        """TEST 2: Hedef vs Gerçek Karşılaştırma"""
        print("\n" + "="*50)
        print("TEST 2: Hedef vs Gerçek Açı")
        print("="*50)

        target_angle = 1.0  # ~57 derece
        print(f"\nHedef: Eklem 0'ı {target_angle:.3f} radyana götür ({math.degrees(target_angle):.1f}°)")
        self.set_joint_position(0, target_angle)

        # Hareket sırasında her 100 adımda durumu göster
        print("\nHareket ediyor...")
        for step in range(500):
            self.simulate(1)
            self.update_joint_history()

            # Her 100 adımda bir rapor ver
            if step % 100 == 0:
                pos, vel, _, _ = self.get_joint_state(0)
                print(f"  Adım {step:3d}: Açı={math.degrees(pos):6.2f}°, Hız={vel:6.3f} rad/s")

        # Final durumu
        pos_final, vel_final, _, _ = self.get_joint_state(0)
        print(f"\n✓ Hareket tamamlandı!")
        print(f"  Hedef açı : {math.degrees(target_angle):.2f}°")
        print(f"  Gerçek açı: {math.degrees(pos_final):.2f}°")
        print(f"  Fark      : {math.degrees(pos_final - target_angle):.2f}°")

        input("\nDevam etmek için ENTER'a basın...")

    def run_test_3(self):
        """TEST 3: İki Eklemin Koordineli Hareketi"""
        print("\n" + "="*50)
        print("TEST 3: İki Eklem Koordineli Hareket")
        print("="*50)

        # Hedefler
        target_0 = 0.8   # ~46 derece
        target_1 = -0.6  # ~-34 derece

        print(f"\nHedefler:")
        print(f"  Eklem 0: {math.degrees(target_0):.1f}°")
        print(f"  Eklem 1: {math.degrees(target_1):.1f}°")

        self.set_joint_position(0, target_0)
        self.set_joint_position(1, target_1)

        print("\nHareket ediyor...")
        for step in range(500):
            self.simulate(1)
            self.update_joint_history()

            if step % 100 == 0:
                pos_0, vel_0, _, _ = self.get_joint_state(0)
                pos_1, vel_1, _, _ = self.get_joint_state(1)
                print(f"  Adım {step:3d}:")
                print(f"    Eklem 0: {math.degrees(pos_0):6.2f}°")
                print(f"    Eklem 1: {math.degrees(pos_1):6.2f}°")

        print("\n✓ Koordineli hareket tamamlandı!")

        input("\nDevam etmek için ENTER'a basın...")

    def run_test_4(self):
        """TEST 4: Canlı İzleme (Gerçek Zamanlı)"""
        print("\n" + "="*50)
        print("TEST 4: Canlı İzleme - Robot Sallanırken")
        print("="*50)
        print("10 saniye boyunca robot sallanacak")
        print("Eklem durumları gerçek zamanlı gösterilecek\n")

        start_time = time.time()
        t = 0

        while time.time() - start_time < 10:  # 10 saniye
            # Sinüzoidal hareket
            target_0 = 0.6 * math.sin(t)
            target_1 = 0.4 * math.cos(t * 1.3)

            self.set_joint_position(0, target_0)
            self.set_joint_position(1, target_1)

            # Gerçek durumu oku
            pos_0, vel_0, _, _ = self.get_joint_state(0)
            pos_1, vel_1, _, _ = self.get_joint_state(1)

            # Ekrana yazdır (aynı satırı güncelle)
            print(f"\rEklem 0: {math.degrees(pos_0):6.1f}° | "
                  f"Eklem 1: {math.degrees(pos_1):6.1f}° | "
                  f"Süre: {time.time()-start_time:.1f}s", end="")

            self.simulate(1)
            self.update_joint_history()
            t += 0.02

        print("\n\n✓ Canlı izleme tamamlandı!")

        input("\nDevam etmek için ENTER'a basın...")

    def run_test_5(self):
        """TEST 5: Son Efektör Takibi"""
        print("\n" + "="*50)
        print("TEST 5: Son Efektör Takibi")
        print("="*50)
        print("Robotun son efektör pozisyonu hareket sırasında takip edilecek\n")

        # Başlangıç pozisyonunu kaydet
        start_pos, _ = self.get_end_effector_position()
        print(f"Başlangıç pozisyonu: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")

        # Hedef pozisyonlar
        targets = [
            [0.5, 0.0, 0.5],
            [0.5, 0.5, 0.5],
            [0.0, 0.5, 0.5],
            [0.0, 0.0, 0.5]
        ]

        for i, target in enumerate(targets):
            print(f"\nHedef {i+1}: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")

            # Basit bir ters kinematik yaklaşımı (sadece demo amaçlı)
            # Gerçek uygulamada daha gelişmiş bir algoritma gerekir
            if target[0] != 0:
                self.set_joint_position(0, math.atan2(target[1], target[0]))
            else:
                self.set_joint_position(0, 0)

            if target[1] != 0:
                self.set_joint_position(1, math.atan2(target[2], math.sqrt(target[0]**2 + target[1]**2)))
            else:
                self.set_joint_position(1, math.pi/2)

            # Hareket et ve bekle
            for step in range(200):
                self.simulate(1)
                self.update_joint_history()

            # Son pozisyonu oku
            end_pos, _ = self.get_end_effector_position()
            print(f"Son pozisyon: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")

            # Hata hesapla
            error = math.sqrt(sum((end_pos[j] - target[j])**2 for j in range(3)))
            print(f"Hata: {error:.3f} m")

        input("\nDevam etmek için ENTER'a basın...")

    def run_test_6(self):
        """TEST 6: Eklem Pozisyon Grafiği"""
        print("\n" + "="*50)
        print("TEST 6: Eklem Pozisyon Grafiği")
        print("="*50)
        print("Önceki testlerdeki eklem hareketleri grafiğe dökülecek\n")

        # Rastgele hareketler yap ve veri topla
        print("Rastgele hareketler yapılıyor...")
        for _ in range(300):
            # Rastgele hedefler
            targets = [np.random.uniform(-1, 1) for _ in range(3)]
            self.set_joint_positions(targets)

            for _ in range(10):
                self.simulate(1)
                self.update_joint_history()

        print("Veri toplandı, grafik çiziliyor...")
        self.plot_joint_positions(range(3))

        input("\nDevam etmek için ENTER'a basın...")

    def run_all_tests(self):
        """Tüm testleri sırayla çalıştırır"""
        self.run_test_1()
        self.run_test_2()
        self.run_test_3()
        self.run_test_4()
        self.run_test_5()
        self.run_test_6()

        # Özet
        print("\n" + "="*50)
        print("TÜM TESTLER TAMAMLANDI")
        print("="*50)
        print("✓ Eklem durumlarını okumayı öğrendik")
        print("✓ Hedef vs gerçek açıları karşılaştırdık")
        print("✓ Gerçek zamanlı izleme yaptık")
        print("✓ Son efektör pozisyonunu takip ettik")
        print("✓ Eklem hareketlerini grafiğe döktük")
        print("\nSonraki adımlar:")
        print("- Daha gelişmiş kinematik hesaplamaları")
        print("- Yol planlama algoritmaları")
        print("- Çoklu robot koordinasyonu")
        print("="*50)

    def disconnect(self):
        """Simülasyon bağlantısını kapatır"""
        p.disconnect()
        print("✓ Simülasyon bağlantısı kapatıldı")


def main():
    """Ana fonksiyon"""
    try:
        # Simülatörü başlat
        sim = RobotSimulator(gui=True)

        # Tüm testleri çalıştır
        sim.run_all_tests()

    except Exception as e:
        print(f"Hata: {e}")
    finally:
        # Bağlantıyı kapat
        if 'sim' in locals():
            sim.disconnect()


if __name__ == "__main__":
    main()
