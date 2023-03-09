"""
========================

Geometry plotter class

========================
"""

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from VLM.geometry.geometry import geometry_generator
from VLM.utility.enums import AERODYNAMIC_SURFACE


class geometry_plotter(geometry_generator):
    """
        Geometry plotter class. This class inherits geometry_generator class, and will generate some plots of the
        geometry. If a path to the .npy geometry is given, it will not call super class geometry_generator and will 
        automatically generate all plots. Otherwise, it will regenerate the geometry (based on the .json input file).
    
    """

    def __init__(self, path_to_data):
        """
        
        :param path_to_data: 
        """

        if '.npy' in path_to_data:
            self.geometry = np.load(path_to_data, allow_pickle=True).item()
        elif '.json' in path_to_data:
            super().__init__(path_to_data)
        self.geometry.pop('b')
        self.geometry.pop('MAC')
        self.geometry.pop('S')

        fig = plt.figure(figsize=(8, 6))
        ax = plt.axes(projection='3d')
        for surface, data in self.geometry.items():
            surface = self._get_aerodynamic_surface(surface)
            contador = 0
            if surface in {AERODYNAMIC_SURFACE.WING, AERODYNAMIC_SURFACE.WING_AILERON, AERODYNAMIC_SURFACE.WING_FLAP}:
                color = 'red'
                label = 'WING'
            elif surface in {AERODYNAMIC_SURFACE.VTP, AERODYNAMIC_SURFACE.VTP_RUDDER}:
                color = 'blue'
                label = 'VTP'
            elif surface in {AERODYNAMIC_SURFACE.HTP, AERODYNAMIC_SURFACE.HTP_ELEVATOR}:
                color = 'green'
                label = 'HTP'

            for i in data['plot']:
                for j in i:
                    for k in j:
                        if contador == 0:
                            ax.plot3D([k[1][0], k[3][0]], [k[1][1], k[3][1]], [k[1][2], k[3][2]], color, label=label)
                            contador += 1
                        else:
                            ax.plot3D([k[1][0], k[3][0]], [k[1][1], k[3][1]], [k[1][2], k[3][2]], color)
                        ax.plot3D([k[0][0], k[1][0]], [k[0][1], k[1][1]], [k[0][2], k[1][2]], color)
                        ax.plot3D([k[3][0], k[2][0]], [k[3][1], k[2][1]], [k[3][2], k[2][2]], color)
                        ax.plot3D([k[2][0], k[0][0]], [k[2][1], k[0][1]], [k[2][2], k[0][2]], color)
                        if surface not in {AERODYNAMIC_SURFACE.VTP, AERODYNAMIC_SURFACE.VTP_RUDDER}:
                            ax.plot3D([k[0][0], k[1][0]], [-k[0][1], -k[1][1]], [k[0][2], k[1][2]], color)
                            ax.plot3D([k[1][0], k[3][0]], [-k[1][1], -k[3][1]], [k[1][2], k[3][2]], color)
                            ax.plot3D([k[3][0], k[2][0]], [-k[3][1], -k[2][1]], [k[3][2], k[2][2]], color)
                            ax.plot3D([k[2][0], k[0][0]], [-k[2][1], -k[0][1]], [k[2][2], k[0][2]], color)

        ax.set_xlabel("X", rotation=180)
        ax.set_ylabel("Y")
        ax.set_zlabel('Z')
        plt.legend()
        ax.view_init(10, 180)
        plt.title('Aircraft Geometry')
        plt.savefig('VLM/aircraft_front_view.eps', bbox_inches='tight', format='eps', dpi=1200)
        ax.view_init(90, 180)
        plt.savefig('VLM/aircraft_top_view.eps', bbox_inches='tight', format='eps', dpi=1200)
        ax.view_init(10, 90)
        plt.savefig('VLM/aircraft_side_view.eps', bbox_inches='tight', format='eps', dpi=1200)


__all__ = ['geometry_plotter']