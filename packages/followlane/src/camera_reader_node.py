#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import yaml
import cv2
import numpy as np
from cv_bridge import CvBridge
import tkinter as tk
from PIL import Image, ImageTk

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # Speichere Pfad zur YAML-Datei für späteres Abspeichern
        self.config_path = 'packages/followlane/config/detect_lane.yaml'

        # Lese Konfiguration aus der YAML-Datei
        with open(self.config_path, 'r') as f:
            text = f.read()
        self.conf = yaml.safe_load(text)

        # Setze den Fahrzeugnamen und Kameratopic anhand der Umgebung
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        # OpenCV ↔ ROS Bridge
        self._bridge = CvBridge()

        # Erzeuge das UI-Fenster mit Reglern
        self.create_window()

        # Abonniere das Kamerabild
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg):
        # Aktualisiere Parameter mit aktuellen Sliderwerten
        self.update_conf()

        # Konvertiere ROS-Bild in OpenCV-Bild
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Je nach Auswahl im Dropdown Menü: Bildverarbeitung
        if self.selected.get() == 'lane_image':
            # Zeichne Polygon zur Anzeige der ROI
            x_alt = 0
            y_alt = 0
            for point in ['top_left','top_right','bottom_left','bottom_right','top_left']:
                x = self.conf['lane_image'][f'{point}_x']
                y = self.conf['lane_image'][f'{point}_y']
                if x_alt != 0 or y_alt != 0:
                    image = cv2.line(image, (x_alt, y_alt), (x, y), (255,255,255), 2)
                x_alt = x
                y_alt = y

        elif self.selected.get() == 'mapping':
            # Zeichne vier Punkte und obere Begrenzungslinie
            x_alt = 0
            y_alt = 0
            for point in ['chess_pt1','chess_pt2','chess_pt3','chess_pt4','chess_pt1']:
                x = self.conf['mapping'][f'{point}_x']
                y = self.conf['mapping'][f'{point}_y']
                if x_alt != 0 or y_alt != 0:
                    image = cv2.line(image, (x_alt, y_alt), (x, y), (255,255,255), 2)
                x_alt = x
                y_alt = y
            y_max, x_max, _ = image.shape
            y_cutoff = int(y_max * self.conf['mapping']['topLimit'])
            image = cv2.line(image, (0, y_cutoff), (x_max, y_cutoff), (255,255,255), 3)

        else:
            # Zeige Maske für gewählten Farbbereich
            hl = self.conf[self.selected.get()]['hl']
            hh = self.conf[self.selected.get()]['hh'] 
            sl = self.conf[self.selected.get()]['sl']
            sh = self.conf[self.selected.get()]['sh']
            vl = self.conf[self.selected.get()]['vl']
            vh = self.conf[self.selected.get()]['vh'] 

            image = cv2.inRange(image, (hl, sl, vl), (hh, sh, vh))

        # Zeige Bild im Fenster
        image = ImageTk.PhotoImage(Image.fromarray(image))
        self.panel.configure(image=image)
        self.panel.image = image

    def update_conf(self):
        """Aktualisiere Konfigurationsdaten basierend auf aktuellen Sliderwerten."""
        try:
            for val in self.conf[self.selected.get()]:
                name = f'{self.selected.get()}_{val}'
                self.conf[self.selected.get()][val] = self.sliders[name].get()
        except Exception as e:
            print(f"❌ Fehler beim Updaten der Konfiguration: {e}")

    def save_conf(self):
        """Speichert die Konfiguration zurück in die YAML-Datei."""
        try:
            with open(self.config_path, 'w') as f:
                yaml.dump(self.conf, f)
            print(f'✅ Konfiguration gespeichert in {self.config_path}')
        except Exception as e:
            print(f'❌ Fehler beim Speichern: {e}')

    def print_conf(self):
        """Gibt aktuelle Konfiguration in der Konsole aus."""
        text = yaml.safe_dump(self.conf)
        print(f'#############\n{text}\n#############')

    def change_menue(self, *args):
        """Wird aufgerufen, wenn anderes Element aus Dropdown ausgewählt wird."""
        self.slider_frame.pack_forget()
        print(f'Selected Menü: {self.selected.get()}')
        self.slider_frame = self.slider_frames[self.selected.get()]
        self.slider_frame.pack()
        self.print_conf()

    def create_window(self):
        """Erzeugt das tkinter-Fenster mit Dropdown und Slidern."""
        self._root = tk.Tk()

        # Schwarzes Startbild anzeigen
        img = ImageTk.PhotoImage(Image.fromarray(np.zeros([480, 640, 3], np.uint8)))
        self.panel = tk.Label(self._root, image=img)
        self.panel.pack(side='bottom')

        # Dropdown-Menü mit allen Bereichen aus YAML (z. B. red, white, lane_image)
        options = [s for s in self.conf]
        self.selected = tk.StringVar(self._root)
        self.selected.set(options[0])
        self.selected.trace("w", lambda *args: self.change_menue(*args))
        self.dropdown = tk.OptionMenu(self._root, self.selected, *options)
        self.dropdown.pack(side='top')

        # Erzeuge Slider für jede Konfigurationsgruppe
        self.sliders = {}
        self.slider_frames = {}
        for option in options:
            frame = tk.Frame(self._root)
            for val in self.conf[option]:
                name = f'{option}_{val}'
                # Unterschiedliche Skalenbereiche je nach Parameter
                if option == 'lane_image':
                    self.sliders[name] = tk.Scale(frame, from_=-100, to=700, orient='horizontal', label=val)
                elif option == 'mapping':
                    y, x, _ = img._PhotoImage__photo.size
                    self.sliders[name] = tk.Scale(frame, from_=0, to=2000, orient='horizontal', label=val)
                else:
                    self.sliders[name] = tk.Scale(frame, from_=0, to=255, orient='horizontal', label=val)

                self.sliders[name].set(self.conf[option][val])
                self.sliders[name].pack(side='left')
            self.slider_frames[option] = frame

        self.slider_frame = self.slider_frames[self.selected.get()]
        self.slider_frame.pack()

    def run(self):
        """Starte die UI und speichere Konfiguration beim Schließen."""
        self._root.mainloop()
        self.update_conf()  # letzte Werte übernehmen
        self.save_conf()    # YAML speichern
        self.print_conf()
        rospy.signal_shutdown('User endet Programm')

if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    node.run()
    rospy.spin()
