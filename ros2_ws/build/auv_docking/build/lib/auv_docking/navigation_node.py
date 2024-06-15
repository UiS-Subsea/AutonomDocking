#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ROVNavigator(Node):
    def __init__(self):
        super().__init__('rov_navigator')
        self.pallet_found = False

    def move_forward(self, distance):
        self.get_logger().info(f"Moving forward {distance} meters.")
        # Legg inn logikk for å oppdatere ROVens posisjon
        self.check_for_pallet()

    def move_backward(self, distance):
        self.get_logger().info(f"Moving backward {distance} meters.")
        # Legg inn logikk for å oppdatere ROVens posisjon
        self.check_for_pallet()

    def move_left(self, distance):
        self.get_logger().info(f"Moving left {distance} meters.")
        # Legg inn logikk for å oppdatere ROVens posisjon
        self.check_for_pallet()

    def move_right(self, distance):
        self.get_logger().info(f"Moving right {distance} meters.")
        # Legg inn logikk for å oppdatere ROVens posisjon
        self.check_for_pallet()

    def check_for_pallet(self):
        # Logikk for å sjekke om pallen er funnet
        # Dette kan inkludere bildebehandling og ArUco-deteksjon
        # Hvis pallen er funnet, sett self.pallet_found til True
        pass

    def search_strategy(self):
        # Fase 1: Rett fremover
        self.move_forward(distance=10)  # 10 meter fremover
        
        # Hvis pallen ikke er funnet, gå til Fase 2
        if not self.pallet_found:
            # Fase 2: "Lawnmower"-mønster på andre halvdel av bassenget
            number_of_passes = int((10 / 2))  # Juster basert på synsfeltet til kameraet
            for _ in range(number_of_passes):
                self.move_left(distance=2)  # Juster denne avstanden basert på kameraets synsfelt og overlapp
                self.move_backward(distance=20)  # 20 meter bakover til startpunktet
                self.move_left(distance=2)  # Juster denne avstanden basert på kameraets synsfelt og overlapp
                self.move_forward(distance=20)  # 20 meter fremover igjen
                
                # Sjekk om pallen er funnet etter hver pass
                if self.pallet_found:
                    break

def main(args=None):
    rclpy.init(args=args)
    navigator = ROVNavigator()
    navigator.search_strategy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
