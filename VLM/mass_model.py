"""
    ==========================
    Aircraft Model Mass
    ==========================

    Mass properties calculation for the aircraft
"""

import json
import numpy as np

from VLM.utility.enums import MASS_CREATOR, AIRCRAFT_PARTS
from VLM.utility.utilities import surface_calculation


class Mass_model(object):
    """
        Mass model class. This class is used to estimate all mass-relative parameters of the aircraft, such us inertia,
        center of gravity location...

    """

    def __init__(self, geometry_path, mass_data_path):
        """
        Constructor method of mass_model class. Initializes geometry class to load or generate the geometry of the
        aircraft and uses all methods defined in the class to generate and save mass properties of the aircraft. It uses
        a lumped mass model to distribute the weight all over the aircraft and then, it calculates the center of gravity
        location, the inertia matrix of the complete model ...

        :param geometry_path: path to geometry already generated
        :type geometry_path: 'path'
        :param mass_data_path: path to mass data of the aircraft
        :type mass_data_path: 'path'

        """
        self.__out_name = geometry_path.split('.')[0]
        self.geometry = np.load(geometry_path, allow_pickle=True).item()

        with open(mass_data_path, 'r') as f:
            data = json.load(f)

        self.mass = {'type': self._get_mass_type(data['mass_type']), "WEIGHT": data['WEIGHT']}
        self._mass_model()
        self.location = data['POSITION']
        self._data = {}
        self._calculate_cg()
        self._inertia()

    def _mass_model(self):
        """
        Method to prepare the mass data of the aircraft. Depending the type included in the file, some operations will
        or will not be done. If a number of resources have been consulted and the user wants to use the mean of all
        data, type must be set to MEAN. Otherwise, if data is certainly known, use DATA

        :return:
        """

        if self.mass['type'] == MASS_CREATOR.MEAN:
            for key, item in self.mass['WEIGHT'].items():
                self.mass['WEIGHT'][key] = sum(item) / len(item)
        elif self.mass['type'] == MASS_CREATOR.DATA:
            f'Nothing to do here, just to get information'
        MTOW = 0
        for key, item in self.mass['WEIGHT'].items():
            MTOW += item
        self.MTOW = MTOW

    def _inertia(self):
        """
        Method to obtain the inertia matrix of the aircraft

        """
        I_xx = 0
        I_yy = 0
        I_zz = 0
        I_xy = 0
        I_xz = 0
        I_yz = 0

        for key, items in self._data.items():
            if key in {AIRCRAFT_PARTS.WING, AIRCRAFT_PARTS.HTP, AIRCRAFT_PARTS.VTP}:
                temp_xx = 0.0
                temp_yy = 0.0
                temp_zz = 0.0
                temp_xz = 0.0
                for i in range(len(items['positions'])):
                    temp_xx += items['mass'][i] * ((items['position'][i][1] - self.cg[1])**2 + (items['position'][i][2]
                                                                                                - self.cg[2])**2)
                    temp_yy += items['mass'][i] * ((items['position'][i][2] - self.cg[2])**2 + (items['position'][i][0]
                                                                                                - self.cg[0])**2)
                    temp_zz += items['mass'][i] * ((items['position'][i][1] - self.cg[1])**2 + (items['position'][i][0]
                                                                                                - self.cg[0])**2)
                    temp_xz += items['mass'][i] * ((items['position'][i][2] - self.cg[2]) + (items['position'][i][0]
                                                                                             - self.cg[0]))
                I_xx += temp_xx
                I_yy += temp_yy
                I_zz += temp_zz
                I_xz += temp_xz
            else:
                I_xx += items['mass'] * ((items['position'][1] - self.cg[1])**2 +
                                         (items['position'][2] - self.cg[2])**2)
                I_yy += items['mass'] * ((items['position'][2] - self.cg[2])**2 +
                                         (items['position'][0] - self.cg[0])**2)
                I_zz += items['mass'] * ((items['position'][1] - self.cg[1])**2 +
                                         (items['position'][0] - self.cg[0])**2)
                I_xz += items['mass'] * ((items['position'][2] - self.cg[2]) + (items['position'][0] - self.cg[0]))

        self.__inertia_matrix = np.array([I_xx, I_yy, I_zz, I_xy, I_xz, I_yz])

        return print(self._generate_output())

    def _calculate_cg(self):
        """
        Method to compute the center of gravity of the total aircraft (Including all parts)
        :return:
        """
        sum_cg = np.zeros((3, 1))
        for key, item in self.location.items():
            mass = 0.0
            data = self._get_aircraft_part(key)
            if data in {AIRCRAFT_PARTS.WING, AIRCRAFT_PARTS.HTP, AIRCRAFT_PARTS.VTP}:
                part = ""
                mass_template = np.array([])
                mass = self.mass['WEIGHT'][item['item'][0]]
                if data == AIRCRAFT_PARTS.WING:
                    part = 'WING'
                elif data == AIRCRAFT_PARTS.HTP:
                    part = 'HTP'
                elif data == AIRCRAFT_PARTS.VTP:
                    part = 'VTP'
                for i in self.geometry[part]['plot']:
                    for j in i:
                        for k in j:
                            if type(k[0]) is list:
                                mass_template = np.append(mass_template, surface_calculation(np.array(k[0]),
                                                                                             np.array(k[1]),
                                                                                             np.array(k[2]),
                                                                                             np.array(k[3])))
                            else:
                                mass_template = np.append(mass_template, surface_calculation(k[0], k[1], k[2], k[3]))

                mass_template = mass_template * mass / mass_template.sum()
                temp = np.array(self.geometry[part]['r_c'])
                temp[:, 0] = temp[:, 0] * mass_template
                temp[:, 1] = temp[:, 1] * mass_template
                temp[:, 2] = temp[:, 2] * mass_template

                sum_cg = sum_cg + np.array([temp[:, 0].sum(), temp[:, 1].sum(), temp[:, 2].sum()])
                self._data[data] = {'mass': mass_template, 'position': self.geometry[part]['r_c']}

            elif data in {AIRCRAFT_PARTS.MAIN_LG, AIRCRAFT_PARTS.AUX_LG}:
                cg = np.array(item['position'])
                sum_cg += cg * self.mass['WEIGHT'][item['item'][0]]
                self._data[data] = {'mass': self.mass['WEIGHT'][item['item'][0]], 'position': cg}

            elif data == AIRCRAFT_PARTS.FUEL:
                cont = 0
                for i, temp in item['tanks']:
                    """
                        Contribution of all fuel to the total cg. If symmetry == False, it is
                        assumed that the position of the tank is given. If symmetry == True,
                        the tank is suppossed to be located on the wing, iterating over the
                        different sections the wind is divided in.
                    """

                    if temp['symmetry']:
                        cg = (self.geometry['WING']['plot'][cont][0][0] + self.geometry['WING']['plot'][cont][0][
                            2]) / 2 + \
                             (self.geometry['WING']['plot'][cont][-1][1] + self.geometry['WING']['plot'][cont][-1][
                                 3]) / 2

                        position = np.array([[cg[:][0], 0.0, cg[:][2]]])
                        sum_cg += self.mass['WEIGHT'][item['item'][0]] * cg * temp['weight_%'] * 2
                        cont += 1
                        self._data[data] = {'mass': 2 * self.mass['WEIGHT'][item['item'][0]] * temp['weight_%'],
                                            'position': position}
                    else:
                        cg = np.array(temp['position'])
                        sum_cg += self.mass['WEIGHT'][item['item'][0]] * cg * temp['weight_%']
                        self._data[data] = {'mass': self.mass['WEIGHT'][item['item'][0]] * temp['weight_%'],
                                            'position': cg}

            elif data == AIRCRAFT_PARTS.ENGINE:
                """
                    Contribution of all elements that are related to the engine to the total cg
                """
                cg = np.array(item['position'])
                for i in item['item']:
                    mass += self.mass['WEIGHT'][i]
                sum_cg[0] += cg[0] * mass
                sum_cg[1] += cg[1] * mass
                sum_cg[2] += cg[2] * mass

                self._data[data] = {'mass': mass, 'position': cg}

            elif data == AIRCRAFT_PARTS.FUSELAGE:
                """
                    Contribution of all elements that are related to fuselage to the total cg.
                    Center of gravity of the fuselage assumed to be in the mid point of it
                """
                cg = np.array([(item['position'][1] - item['position'][0]) / 2, 0.0, 0.0])
                for i in item['item']:
                    mass += self.mass['WEIGHT'][i]
                sum_cg[0] += cg[0] * mass
                sum_cg[1] += cg[1] * mass
                sum_cg[2] += cg[2] * mass
                self._data[data] = {'mass': mass, 'position': cg}

        self.cg = sum_cg / self.MTOW

    def _generate_output(self):
        """
        Method to generate the output of the inertia matrix of the aircraft
        :return:
        """

        output = {'I_xx': self.__inertia_matrix[0], 'I_yy': self.__inertia_matrix[1], 'I_zz': self.__inertia_matrix[2],
                  'I_xy': self.__inertia_matrix[3], 'I_xz': self.__inertia_matrix[4], 'I_yz': self.__inertia_matrix[5]}
        path = 'VLM/inertia' + self.__out_name + '.json'
        with open(path, 'w') as f:
            json.dump(output, f)
        inertia_matrix = np.array([[self.__inertia_matrix[0], self.__inertia_matrix[3], self.__inertia_matrix[4]],
                                   [self.__inertia_matrix[3], self.__inertia_matrix[1], self.__inertia_matrix[5]],
                                   [self.__inertia_matrix[4], self.__inertia_matrix[5], self.__inertia_matrix[2]]])

        return f'Inertia matrix\n: {inertia_matrix}'

    @staticmethod
    def _get_mass_type(data):
        """
        Method to get mass type enumerator

        :param data: mass data type
        :type data: string
        :return: enums.MASS_CREATOR object
        """

        return getattr(MASS_CREATOR, data)

    @staticmethod
    def _get_aircraft_part(data):
        """
        Method to get aircraft part enumerator

        :param data: aircraft part
        :type data: string
        :return: enums.AIRCRAFT_PARTS object
        """

        return getattr(AIRCRAFT_PARTS, data)
