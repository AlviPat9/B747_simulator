"""
Geometry generator class

"""

from VLM.utility.enums import *
from VLM.utility.utilities import deg2rad, norm, calculate_angles, cos, sin
import numpy as np
import json


class geometry_generator(object):
    """
    Aircraft geometry generator class
    """
    output = {}
    __s = 0
    __y = 0
    __x = 0
    __mac = 0
    _control_surface = None
    __VTP = False
    __check = 0

    def __init__(self, path_to_geom):
        """
        :param path_to_geom:
        """

        with open(path_to_geom, 'r') as f:
            geometry = json.load(f)

        for name, keys in geometry.items():
            self.reset_vars()

            keys['type'] = self._get_aerodynamic_surface(name)
            keys['name'] = name
            self._set_calculation(keys)
            if (keys['type'] in {AERODYNAMIC_SURFACE.WING,
                                 AERODYNAMIC_SURFACE.WING_FLAP, AERODYNAMIC_SURFACE.WING_AILERON}) & (self.__check == 0):
                self.output['S'] = keys['S_ref']
                self.output['b'] = keys['b']
                self.output['MAC'] = self.__mac
                self.__check = 1
        if self.__check == 0:
            self.output['S'] = keys['S_ref']
            self.output['b'] = keys['b']
            self.output['MAC'] = self.__mac
        self.__save_output(path_to_geom)
        self.geometry = self.output

    def _set_calculation(self, data):
        """
        :param data:
        :return:
        """
        if 'sweep' in data:
            self.sweep_matrix = self._rot_matrix(-data['sweep'], self._get_rot_matrix_type('Z'))
            if data['sweep'] > 45.0:
                data['b_sweep'] = data['b'] / cos(deg2rad(25))
            else:
                data['b_sweep'] = data['b'] / cos(deg2rad(data['sweep']))
        else:
            data['sweep_rot'] = self._rot_matrix(0.0, self._get_rot_matrix_type('EYE'))

        if 'dihedral' in data:
            self.dihedral_matrix = self._rot_matrix(-data['dihedral'], self._get_rot_matrix_type('X'))
            if data['dihedral'] >= 45.0:
                data['b_dih'] = data['b'] / sin(deg2rad(data['dihedral']))
            else:
                data['b_dih'] = data['b'] / cos(deg2rad(data['dihedral']))
        else:
            self.dihedral_matrix = self._rot_matrix(0.0, self._get_rot_matrix_type('EYE'))

        data['d_sweep'] = ((data['b_sweep'] / 2) ** 2 - (data['b'] / 2) ** 2) ** 0.5
        data['d_dih'] = ((data['b_dih'] / 2) ** 2 - (data['b'] / 2) ** 2) ** 0.5
        if data['type'] in {AERODYNAMIC_SURFACE.WING, AERODYNAMIC_SURFACE.HTP}:
            self._generate_name(data)
            self._generator_wing(data)
        elif data['type'] in {AERODYNAMIC_SURFACE.WING_FLAP}:
            for flap in data['control_surface']['deflection']:
                self._generate_name(data, flap)
                self._generator_wing(data, 1.0 + flap / 100)
        elif data['type'] in {AERODYNAMIC_SURFACE.HTP_ELEVATOR, AERODYNAMIC_SURFACE.WING_AILERON}:
            self._control_surface = {'type': self._get_aerodynamic_control_surface(data['control_surface']['type']),
                                     'deflection': data['control_surface']['deflection'],
                                     "position": data['control_surface']['position']}
            for c_surface in self._control_surface['deflection']:
                self._control_surface['def'] = calculate_angles(c_surface)
                self._control_surface['rot'] = self._rot_matrix(-c_surface, self._get_rot_matrix_type('Y'))
                self._generate_name(data, c_surface)
                self._generator_wing(data, 1.0 + c_surface / 100)
            self._control_surface = None
            print('Create method for control surfaces')
        elif data['type'] == AERODYNAMIC_SURFACE.VTP:
            self.__VTP = True
            self._generate_name(data)
            self._generator_wing(data)
            self.__VTP = False
        elif data['type'] == AERODYNAMIC_SURFACE.VTP_RUDDER:
            self.__VTP = True
            self._control_surface = {'type': self._get_aerodynamic_control_surface(data['control_surface']['type']),
                                     'deflection': data['control_surface']['deflection'],
                                     "position": data['control_surface']['position']}
            for c_surface in self._control_surface['deflection']:
                self._control_surface['def'] = calculate_angles(c_surface)
                self._control_surface['rot'] = self._rot_matrix(-c_surface, self._get_rot_matrix_type('Y'))
                self._generate_name(data, c_surface)
                # self._generator_wing(data, 1.0 + c_surface / 100)
            self._control_surface = None
            self.__VTP = False

    def _generator_wing(self, data, flap=0.0):
        """
        :param data:
        :param flap:
        :return:
        """
        r_a = np.array([])
        r_b = np.array([])
        r_c = np.array([])
        n = np.array([])
        p_to_plot = []

        r_a_temp = np.array([])
        r_b_temp = np.array([])
        r_c_temp = np.array([])
        n_temp = np.array([])
        span_ratio_0 = 0.0
        root_l = 0.0
        self._get_ref(data)

        for i in data['Section']:
            root_chord = data['c_0']
            if data['type'] == AERODYNAMIC_SURFACE.WING_FLAP:
                root_chord *= flap

            P_1 = np.array(
                [root_chord * (
                        0.25 - 0.25 * data['Section'][i]['taper_0'] + data['d_sweep'] / root_chord * span_ratio_0),
                 data['b'] / 2 * span_ratio_0,
                 data['d_dih'] * span_ratio_0]
            )

            P_2 = np.array(
                [root_chord * (0.25 - 0.25 * data['Section'][i]['taper'] + data['d_sweep'] / root_chord *
                               data['Section'][i]['span_ratio']),
                 data['b'] / 2 * data['Section'][i]['span_ratio'],
                 data['d_dih'] * data['Section'][i]['span_ratio']]
            )

            P_3 = np.array(
                [root_chord * (
                        0.25 + 0.75 * data['Section'][i]['taper_0'] + data['d_sweep'] / root_chord * span_ratio_0),
                 data['b'] / 2 * span_ratio_0,
                 data['d_dih'] * span_ratio_0]
            )

            P_4 = np.array(
                [root_chord * (0.25 + 0.75 * data['Section'][i]['taper'] + data['d_sweep'] / root_chord *
                               data['Section'][i]['span_ratio']),
                 data['b'] / 2 * data['Section'][i]['span_ratio'],
                 data['d_dih'] * data['Section'][i]['span_ratio']]
            )

            self.mac_launch(P_1, P_2, P_3, P_4, data['b'], data['Section'][i])
            span_ratio_0 = data['Section'][i]['span_ratio']

            airfoil = self._airfoil_properties(data['Section'][i]['airfoil'])

            if 'control_surface' in data['Section'][i]:
                if self._control_surface['type'] == \
                        self._get_aerodynamic_control_surface(data['Section'][i]['control_surface']['type']):
                    r_a_temp, r_b_temp, r_c_temp, n_temp, p_to_plot_temp = self._section_(P_1, P_2, P_3, P_4,
                                                                                          data['Section'][i]['Nx'],
                                                                                          data['Section'][i]['Ny'],
                                                                                          airfoil, data['apex_loc'],
                                                                                          True)
                else:
                    r_a_temp, r_b_temp, r_c_temp, n_temp, p_to_plot_temp = self._section_(P_1, P_2, P_3, P_4,
                                                                                          data['Section'][i]['Nx'],
                                                                                          data['Section'][i]['Ny'],
                                                                                          airfoil, data['apex_loc'])
            else:
                r_a_temp, r_b_temp, r_c_temp, n_temp, p_to_plot_temp = self._section_(P_1, P_2, P_3, P_4,
                                                                                      data['Section'][i]['Nx'],
                                                                                      data['Section'][i]['Ny'],
                                                                                      airfoil, data['apex_loc'])
            r_a = np.append(r_a, r_a_temp)
            r_b = np.append(r_b, r_b_temp)
            r_c = np.append(r_c, r_c_temp)
            n = np.append(n, n_temp)
            p_to_plot.append(p_to_plot_temp)

        n_elements = int(len(r_a) / 3)
        r_a = r_a.reshape(n_elements, 3)
        r_b = r_b.reshape(n_elements, 3)
        r_c = r_c.reshape(n_elements, 3)
        n = n.reshape(n_elements, 3)

        return self.generate_output(r_a, r_b, r_c, n, data['S_ref'], data['b'], p_to_plot)

    def _section_(self, P_1, P_2, P_3, P_4, nx, ny, airfoil, apex_loc, deflection=False):
        """
        :param P_1:
        :param P_2:
        :param P_3:
        :param P_4:
        :param nx:
        :param ny:
        :param airfoil:
        :param apex_loc:
        :param deflection:
        :return:
        """
        s_1_x = np.linspace(P_1[0], P_2[0], ny + 1)
        s_1_y = np.linspace(P_1[1], P_2[1], ny + 1)
        s_1_z = np.linspace(P_1[2], P_2[2], ny + 1)

        s_2_x = np.linspace(P_3[0], P_4[0], ny + 1)
        s_2_y = np.linspace(P_3[1], P_4[1], ny + 1)
        s_2_z = np.linspace(P_3[2], P_4[2], ny + 1)

        r_a = np.array([])
        r_b = np.array([])
        r_c = np.array([])
        n = np.array([])
        p_to_plot = []
        for i in range(len(s_1_x) - 1):
            left_seg = np.array(
                [np.linspace(s_1_x[i], s_2_x[i], nx + 1),
                 np.linspace(s_1_y[i], s_2_y[i], nx + 1),
                 np.linspace(s_1_z[i], s_2_z[i], nx + 1)
                 ])
            right_seg = np.array(
                [np.linspace(s_1_x[i + 1], s_2_x[i + 1], nx + 1),
                 np.linspace(s_1_y[i + 1], s_2_y[i + 1], nx + 1),
                 np.linspace(s_1_z[i + 1], s_2_z[i + 1], nx + 1)
                 ])

            r_a_temp, r_b_temp, r_c_temp, n_temp, p_to_plot_temp = self._calculate_hv_points(left_seg, right_seg,
                                                                                             s_2_x[i] - s_1_x[i],
                                                                                             s_2_x[i + 1] - s_1_x[
                                                                                                 i + 1],
                                                                                             airfoil,
                                                                                             apex_loc,
                                                                                             deflection)

            r_a = np.append(r_a, r_a_temp)
            r_b = np.append(r_b, r_b_temp)
            r_c = np.append(r_c, r_c_temp)
            n = np.append(n, n_temp)
            p_to_plot.append(p_to_plot_temp)

        return r_a, r_b, r_c, n, p_to_plot

    def _calculate_hv_points(self, left, right, root_l, root_r, airfoil, apex_loc, deflection):
        """
        :param left:
        :param right:
        :param root_l:
        :param root_r:
        :param airfoil:
        :return:
        """

        p_plot = []
        r_a = []
        r_b = []
        r_c = []
        n = []
        if deflection:
            location = (left[0] - left[0][0]) / root_l
            location = min(np.where(location >= self._control_surface['position'])[0])
        else:
            location = -1
        for i in range(len(left[0]) - 1):
            P1 = [left[0][i], left[1][i],
                  root_l * self._get_z(airfoil, (left[0][i] - left[0][0]) / root_l) + left[2][i]]

            P3 = [left[0][i + 1], left[1][i + 1],
                  root_l * self._get_z(airfoil, (left[0][i + 1] - left[0][0]) / root_l) + left[2][i + 1]]

            P2 = [right[0][i], right[1][i],
                  root_r * self._get_z(airfoil, (right[0][i] - right[0][0]) / root_r) + right[2][i]]

            P4 = [right[0][i + 1], right[1][i + 1],
                  root_r * self._get_z(airfoil, (right[0][i + 1] - right[0][0]) / root_r) + right[2][i + 1]]

            if deflection:
                if i >= location:
                    P1 = self._control_surface['rot'].dot([P1[0] - left[0][location], P1[1] - left[1][location],
                                                           P1[2] - left[2][location]])
                    P2 = self._control_surface['rot'].dot([P2[0] - right[0][location], P2[1] - right[1][location],
                                                           P2[2] - right[2][location]])
                    P3 = self._control_surface['rot'].dot([P3[0] - left[0][location], P3[1] - left[1][location],
                                                           P3[2] - left[2][location]])
                    P4 = self._control_surface['rot'].dot([P4[0] - right[0][location], P4[1] - right[1][location],
                                                           P4[2] - right[2][location]])
                    P1[0] += left[0][location]
                    P1[1] += left[1][location]
                    P1[2] += left[2][location]

                    P2[0] += right[0][location]
                    P2[1] += right[1][location]
                    P2[2] += right[2][location]

                    P3[0] += left[0][location]
                    P3[1] += left[1][location]
                    P3[2] += left[2][location]

                    P4[0] += right[0][location]
                    P4[1] += right[1][location]
                    P4[2] += right[2][location]

            if self.__VTP:
                P1 = self.dihedral_matrix.dot([P1[0] - self.__P1[0][i], P1[1], P1[2] - self.__P1[1][i]])
                P3 = self.dihedral_matrix.dot([P3[0] - self.__P1[0][i + 1], P3[1], P3[2] - self.__P1[1][i + 1]])
                P2 = self.dihedral_matrix.dot([P2[0] - self.__P1[0][i], P2[1], P2[2] - self.__P1[1][i]])
                P4 = self.dihedral_matrix.dot([P4[0] - self.__P1[0][i + 1], P4[1], P4[2] - self.__P1[1][i + 1]])
                P1[0] += self.__P1[0][i]
                P1[2] += self.__P1[1][i]
                P2[0] += self.__P1[0][i]
                P2[2] += self.__P1[1][i]
                P3[0] += self.__P1[0][i + 1]
                P3[2] += self.__P1[1][i + 1]
                P4[0] += self.__P1[0][i + 1]
                P4[2] += self.__P1[1][i + 1]
            P1[0] += apex_loc[0]
            P1[1] += apex_loc[1]
            P1[2] += apex_loc[2]

            P2[0] += apex_loc[0]
            P2[1] += apex_loc[1]
            P2[2] += apex_loc[2]

            P3[0] += apex_loc[0]
            P3[1] += apex_loc[1]
            P3[2] += apex_loc[2]

            P4[0] += apex_loc[0]
            P4[1] += apex_loc[1]
            P4[2] += apex_loc[2]

            p_plot.append([P1, P2, P3, P4])
            ra = np.array(
                [(P3[0] - P1[0]) * 0.25 + P1[0], (P3[1] - P1[1]) * 0.25 + P1[1], (P3[2] - P1[2]) * 0.25 + P1[2]])
            rb = np.array(
                [(P4[0] - P2[0]) * 0.25 + P2[0], (P4[1] - P2[1]) * 0.25 + P2[1], (P4[2] - P2[2]) * 0.25 + P2[2]])
            rc = np.array(
                [P1[0] + (P2[0] - P1[0]) * 0.5 + (P3[0] + (P4[0] - P3[0]) * 0.5 - P1[0] - (P2[0] - P1[0]) * 0.5) * 0.75,
                 P1[1] + (P2[1] - P1[1]) * 0.5 + (P3[1] + (P4[1] - P3[1]) * 0.5 - P1[1] - (P2[1] - P1[1]) * 0.5) * 0.75,
                 P1[2] + (P2[2] - P1[2]) * 0.5 + (
                         P3[2] + (P4[2] - P3[2]) * 0.5 - P1[2] - (P2[2] - P1[2]) * 0.5) * 0.75])

            n.append(self._normal(ra, rb, rc))
            r_a.append(ra)
            r_b.append(rb)
            r_c.append(rc)

        return r_a, r_b, r_c, n, p_plot

    def mac_launch(self, P1, P2, P3, P4, b, data):
        """
        :param b:
        :param data:
        :param P1:
        :param P2:
        :param P3:
        :param P4:
        :return:
        """
        surface = norm(np.cross([P3[0] - P1[0], P3[1] - P1[1]], [P2[0] - P1[0], P2[1] - P1[1]])) / 2 + \
                  norm(np.cross([P1[0] - P2[0], P1[1] - P2[1]], [P4[0] - P2[0], P4[1] - P2[1]])) / 2
        tap = abs(P2[0] - P4[0]) / abs(P3[0] - P1[0])
        mac = 2 / 3 * abs(P3[0] - P1[0]) * (1 + tap + tap ** 2) / (1 + tap)
        y = b / 2 * data['span_0'] + b / 2 * (1 + 2 * tap) / (3 + 3 * tap)
        x_le = P1[0] + (P2[0] - P1[0]) * (1 + 2 * tap) / (3 + 3 * tap)
        self.__s += surface
        self.__mac += mac * surface
        self.__y += y * surface
        self.__x += x_le * surface

    def reset_vars(self):
        """
        :return:
        """
        self.__s = 0
        self.__y = 0
        self.__x = 0
        self.__mac = 0

    def generate_output(self, r_a, r_b, r_c, n, S, b, plot):
        """
        :param r_a:
        :param r_b:
        :param r_c:
        :param n:
        :param S:
        :param b:
        :param plot:
        :return:
        """
        if not self.__VTP:
            r_a, r_b, r_c, n = self._wing_symetry(r_a, r_b, r_c, n)

        self.output[self.name] = {'r_a': r_a, 'r_b': r_b, 'r_c': r_c, 'n': n, 'MAC': self.__mac / self.__s,
                                  'x_mac': self.__x / self.__s, 'y_mac': self.__y / self.__s, 'len': len(r_a),
                                  'r_ref': np.array([0.0, 0.0, 0.0]), 'S': S, 'b': b,
                                  'plot': plot, 'S_surface': self.__s}

    def _generate_name(self, data, position=None):
        """
        :param data:
        :return:
        """
        if data['type'] in {AERODYNAMIC_SURFACE.VTP, AERODYNAMIC_SURFACE.HTP, AERODYNAMIC_SURFACE.WING}:
            self.name = data['name']
        elif data['type'] == AERODYNAMIC_SURFACE.WING_FLAP:
            self.name = data['name']
            self._type = {'type': self._control_surface['type'],
                          "deflection (deg)": self._control_surface['deflection']}
        elif data['type'] == AERODYNAMIC_SURFACE.WING_AILERON:
            self.name = data['name']
            self._type = {'type': self._control_surface['type'],
                          "deflection (deg)": self._control_surface['deflection']}
        elif data['type'] == AERODYNAMIC_SURFACE.HTP_ELEVATOR:
            self.name = data['name']
            self._type = {'type': self._control_surface['type'],
                          "deflection (deg)": self._control_surface['deflection']}
        elif data['type'] == AERODYNAMIC_SURFACE.VTP_RUDDER:
            self.name = data['name']
            self._type = {'type': self._control_surface['type'],
                          "deflection (deg)": self._control_surface['deflection']}

    def __save_output(self, path):
        """
        Function to save geometry data and plot it on a figure
        :param path:
        :return:
        """

        name = path.split('/')[-1].split('.')[0]
        # np.save('VLM/' + name + '.npy', self.output)
        np.save(name + '.npy', self.output)

    def _rotate(self, P):
        """
        :param P:
        :param marker:
        :return:
        """

        P[0] -= self.__P1[0]
        P[1] -= self.__P1[1]
        P[2] -= self.__P1[2]
        P = self.dihedral_matrix.dot(self.sweep_matrix.dot(P))
        P[0] += self.__P1[0]
        P[1] += self.__P1[1]
        P[2] += self.__P1[2]
        return P

    def _get_ref(self, data):
        """
        :param data:
        :return:
        """
        P = np.linspace(0.0, data['c_0'], data['Section']['Section 1']['Nx'] + 1)
        airfoil = self._airfoil_properties(data['Section']['Section 1']['airfoil'])
        a = []
        for i in P:
            a.append(data['c_0'] * self._get_z(airfoil, i / data['c_0']))
        self.__P1 = [P, a]

    @staticmethod
    def _wing_symetry(r_a, r_b, r_c, n):
        """
        :param r_a:
        :param r_b:
        :param r_c:
        :param n:
        :return:
        """
        a = r_a.copy()
        a[:, 1] = a[:, 1] * -1
        r_a = np.append(r_a, a)
        a = r_b.copy()
        a[:, 1] = a[:, 1] * -1
        r_b = np.append(r_b, a)
        a = r_c.copy()
        a[:, 1] = a[:, 1] * -1
        r_c = np.append(r_c, a)
        a = n.copy()
        a[:, 1] = a[:, 1] * -1
        n = np.append(n, a)
        div = int(len(r_a) / 3)

        r_a = r_a.reshape(div, 3)
        r_b = r_b.reshape(div, 3)
        r_c = r_c.reshape(div, 3)
        n = n.reshape(div, 3)

        return r_a, r_b, r_c, n

    @staticmethod
    def _normal(ra, rb, rc):
        """
        :param ra:
        :param rb:
        :param rc:
        :return:
        """
        n = np.cross((rc - ra), (rb - rc))
        n = n / norm(n)
        return n

    @staticmethod
    def _get_z(airfoil, x):
        """
        :param airfoil:
        :return:
        """

        if airfoil['type'] == AIRFOIL_TYPE.NACA_5_DIGIT_STD:
            if 0.0 <= x < airfoil['r']:
                z = airfoil['k1'] / 6 * (
                        x ** 3 - 3 * x ** 2 * airfoil['r'] + airfoil['r'] ** 2 * x * (3 - airfoil['r']))
            elif airfoil['r'] <= x <= 1.0:
                z = airfoil['k1'] * airfoil['r'] ** 3 / 6.0 * (1.0 - x)
            else:
                z = 0.0
                raise Exception('Out of bounds')

        return z

    @staticmethod
    def _get_aerodynamic_surface(surface):
        """
        :param surface:
        :return:
        """
        return getattr(AERODYNAMIC_SURFACE, surface)

    @staticmethod
    def _get_aerodynamic_control_surface(surface):
        """
        :param surface:
        :return:
        """
        return getattr(CONTROL_SURFACES, surface)

    @staticmethod
    def _get_rot_matrix_type(axis):
        """
        :param axis:
        :return:
        """
        return getattr(ROTATION_AXIS, axis)

    @staticmethod
    def _airfoil_properties(airfoil_data):
        """
        :param airfoil_data:
        :return:
        """
        if airfoil_data['type'] == 'NACA':
            if len(airfoil_data['code']) == 4:
                print('4 digit airfoil code')
            elif len(airfoil_data['code']) == 5:
                # print('5 digit airfoil code')
                # print('CODE:', airfoil_data['Code'])
                # print('LPQXX')
                CL = int(airfoil_data['code'][0]) * (3 / 20)
                M_chamber = int(airfoil_data['code'][1]) / 20
                camber_line = int(airfoil_data['code'][2])
                thickness = int(airfoil_data['code'][3:]) / 100
                if camber_line == 0:
                    # print('Standard Camber')
                    k1k2 = 0
                    r = np.interp(M_chamber, [0.05, 0.1, 0.15, 0.2, 0.25], [0.0580, 0.1260, 0.2025, 0.290, 0.3910])
                    k1 = np.interp(M_chamber, [0.05, 0.1, 0.15, 0.2, 0.25], [361.4, 51.640, 15.957, 6.643, 3.230])
                    airfoil = AIRFOIL_TYPE.NACA_5_DIGIT_STD
                else:
                    print('Reflex Camber')
                    k1k2 = np.interp(M_chamber, [0.1, 0.15, 0.2, 0.25], [0.000764, 0.00677, 0.0303, 0.1355])
                    r = np.interp(M_chamber, [0.1, 0.15, 0.2, 0.25], [0.13, 0.217, 0.3180, 0.4410])
                    k1 = np.interp(M_chamber, [0.1, 0.15, 0.2, 0.25], [51.990, 15.793, 6.520, 3.191])
                    airfoil = AIRFOIL_TYPE.NACA_5_DIGIT_RFX

        return {'r': r, 'k1': k1, 'k1k2': k1k2, 'type': airfoil}

    @staticmethod
    def _rot_matrix(ang, axis):
        """

        :param ang:
        :param axis:
        :return:
        """
        ang = deg2rad(ang)
        if axis == ROTATION_AXIS.X:
            rot_matrix = np.array([[1.0, 0.0, 0.0],
                                   [0.0, np.cos(ang), np.sin(ang)],
                                   [0.0, -np.sin(ang), np.cos(ang)]
                                   ])
        elif axis == ROTATION_AXIS.Y:
            rot_matrix = np.array([[np.cos(ang), 0.0, np.sin(ang)],
                                   [0.0, 1.0, 0.0],
                                   [-np.sin(ang), 0.0, np.cos(ang)]
                                   ])
        elif axis == ROTATION_AXIS.Z:
            rot_matrix = np.array([[np.cos(ang), -np.sin(ang), 0.0],
                                   [np.sin(ang), np.cos(ang), 0.0],
                                   [0.0, 0.0, 1.0]
                                   ])
        elif axis == ROTATION_AXIS.EYE:
            rot_matrix = np.eye(3, 3)

        return rot_matrix


__all__ = ['geometry_generator']
