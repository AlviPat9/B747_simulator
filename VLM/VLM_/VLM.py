"""
 ============================
  VORTEX LATTICE METHOD (VLM)
 ============================

  VLM implementation.

"""

import numpy as np
import json
from VLM.utility.utilities import deg2rad, calculate_angles, norm, pi, cross, sin, cos


class VLM(object):
    """
    Vortex Lattice Method (VLM)
    This class is a numerical solution implementation of the general 3D lifting surface theory. It is a 3D potential
    flow calculation method. Used to obtain initial structural load estimations and to provide the trim state values and
    stability derivatives for the linearized force and moment equations.
    """
    def __init__(self, geometry):
        """
        Constructor method for VLM object
        """
        self.geometry = None
        self.b = geometry.pop('b')
        self.S = geometry.pop('S')
        self.MAC = geometry.pop('MAC')
        
    def launch_calculation(self, geometry, surface, u_inf, rho, omega=[0.0, 0.0, 0.0], alpha=0.0, beta=0.0):
        """
        :param geometry:
        :param surface:
        :param u_inf:
        :param omega:
        :param alpha:
        :param beta:
        :param rho:
        :return:
        """
        self.geometry = geometry
        alpha = calculate_angles(alpha)
        beta = calculate_angles(beta)
        self._kernel(surface)
        self._vortex(surface, u_inf, omega, alpha, beta)
        
        return self.__output(self._near_field(surface, u_inf, omega, alpha, beta, rho))

    def _kernel(self, geometry):
        """
        :param geometry:
        :return:
        """
        A = np.zeros((self.geometry[geometry]['len'], self.geometry[geometry]['len']))

        data = self.geometry[geometry]
        x = np.array([1.0, 0.0, 0.0])

        for i in range(data['len']):
            rc = data['r_c'][i]
            n = data['n'][i]
            for j in range(data['len']):
                # http://www.dept.aoe.vt.edu/~mason/Mason_f/CAtxtChap6.pdf
                # TODO: Follow page 18-20
                # ra = data['r_a'][j]
                # rb = data['r_b'][j]
                # r0 = rb - ra
                # r1 = rc - ra
                # r2 = rc - rb
                # Vc = cross(r1, r2) / norm(cross(r1, r2)) ** 2 * (r0.dot(r1) / norm(r1) - r0.dot(r2) / norm(r2))
                # Va = np.array([0.0, r1[2], - r1[1]]) * (1.0 + r1[0] / norm(r1)) / (r1[2] ** 2 + r1[1] ** 2)
                # Vb = - np.array([0.0, r2[2], - r2[1]]) * (1.0 + r2[0] / norm(r2)) / (r2[2] ** 2 + r2[1] ** 2)
                # Marl drela
                r_a = data['r_a'][j]
                r_b = data['r_b'][j]
                a = rc - r_a
                b = rc - r_b
                Vc = cross(a, b) * (1 / norm(a) + 1 / norm(b)) / (
                        norm(a) * norm(b) + a.dot(b))
                Va = cross(a, x) / (norm(a) - norm(a) * a.dot(x))
                Vb = - cross(b, x) / (norm(b) - norm(b) * b.dot(x))
                # Aerodynamics for engineers
                # r_a = data['r_a'][j]
                # r_b = data['r_b'][j]
                # r0 = r_b - r_a
                # r1 = rc - r_a
                # r2 = rc - r_b
                #
                # Vc = cross(r1, r2) / (norm(cross(r1, r2)) ** 2) * (r0.dot(r1) / norm(r1) - r0.dot(r2) / norm(r2))
                # Va = np.array([0.0, r1[2], -r1[1]]) * (1.0 + r1[0] / norm(r1)) / (r1[2] ** 2 + r1[1] ** 2)
                # Vb = np.array([0.0, r2[2], -r2[1]]) * (1.0 + r2[0] / norm(r2)) / (r2[2] ** 2 + r2[1] ** 2)
                A[i, j] = np.dot(Vc + Va + Vb, n) / (4 * pi)
        self.A_ij = A  # the first n points are the effect of each hv on panel 1

    def _vortex(self, name, u_inf, omega, alpha, beta):
        """
        :param name:
        :param u_inf:
        :param omega:
        :param alpha:
        :param beta:
        :return:
        """
        V_inf = np.zeros(self.geometry[name]['len'])
        U = u_inf * np.array([-alpha[0] * beta[0], beta[1], -alpha[1] * beta[0]])
        omega = np.array(omega)
        for i in range(self.geometry[name]['len']):
            rc = self.geometry[name]['r_c'][i]
            n = self.geometry[name]['n'][i]
            V_inf[i] = np.dot(- (U + np.cross(omega, rc)), n)

        self.gamma = np.linalg.solve(self.A_ij, V_inf)

    def _near_field(self, name, u_inf, omega, alpha, beta, rho):
        """
        :param name:
        :param u_inf:
        :param omega:
        :param alpha:
        :param beta:
        :return:
        """
        data = self.geometry[name]
        Vi = np.zeros((self.geometry[name]['len'], 3))
        Vj = self.__Vj(name)
        U = u_inf * np.array([-alpha[0] * beta[0], beta[1], -alpha[1] * beta[0]])
        omega = np.array(omega)

        for i in range(data['len']):
            ri = (data['r_a'][i] + data['r_b'][i]) / 2.0
            Vi[i] = - (U + cross(omega, ri))
            for j in range(data['len']):
                Vi[i] += self.gamma[j] * Vj[i][j]

        F = np.zeros((data['len'], 3))
        F_sum = np.zeros(3)
        M = np.zeros((data['len'], 3))
        M_sum = np.zeros(3)
        for i in range(data['len']):
            li = data['r_b'][i] - data['r_a'][i]
            ri = (data['r_a'][i] + data['r_b'][i]) / 2.0
            F[i] = rho * cross(Vi[i], li) * self.gamma[i]
            # F_sum += F[i]
            M[i] = cross((ri - data['r_ref']), F[i])
            # M_sum += M[i]
        F_sum = np.array([F[:, 0].sum(), F[:, 1].sum(), F[:, 2].sum()])
        M_sum = np.array([M[:, 0].sum(), M[:, 1].sum(), M[:, 2].sum()])
        rotation_matrix = np.array([[alpha[0], 0, alpha[1]], [0.0, 1.0, 0.0], [-alpha[1], 0.0, alpha[0]]])
        c_f = np.dot(rotation_matrix, F_sum) / (0.5 * rho * u_inf ** 2 * self.S)
        c_M = np.dot(rotation_matrix, [-M_sum[0] / self.b, -M_sum[1] / self.MAC, M_sum[2] / self.b]) / (0.5 * rho * u_inf ** 2 * self.S)

        if beta[0] != 1.0:
            c_f[0] = -F_sum.dot(np.array([-alpha[0] * beta[0], beta[1], -alpha[1] * beta[0]]))

        return c_f, c_M

    def __Vj(self, name):
        """
        :param name:
        :return:
        """
        data = self.geometry[name]
        Vj = np.zeros((data['len'], data['len'], 3))
        x = np.array([1.0, 0.0, 0.0])

        for i in range(data['len']):
            ri = (data['r_b'][i] + data['r_a'][i]) / 2.0
            for j in range(data['len']):

                # ra = data['r_a'][j]
                # rb = data['r_b'][j]
                # r1 = ri - ra
                # r2 = ri - rb
                # Va = np.array([0.0, r1[2], -r1[1]]) * (1.0 + r1[0] / norm(r1)) / (r1[2] ** 2 + r1[1] ** 2)
                # Vb = np.array([0.0, r2[2], -r2[1]]) * (1.0 + r2[0] / norm(r2)) / (r2[2] ** 2 + r2[1] ** 2)

                r_a = data['r_a'][j]
                r_b = data['r_b'][j]
                a = ri - r_a
                b = ri - r_b

                Va = np.cross(a, x) / (norm(a) - norm(a) * a.dot(x))
                Vb = - np.cross(b, x) / (norm(b) - norm(b) * b.dot(x))
                # # Mark Drela
                Vj[i][j] = (Va + Vb) / (4 * pi)
        return Vj
    
    @staticmethod
    def __output(cf):
        """
        Generate output dictionary
        
        :param f:
        :return
        """
        f = cf[0]
        m = cf[1]
        name_dict = {0: "_x", 1: "_y", 2: "z"}
        output = {}
        
        for i in range(3):
            output["c_f" + name_dict[i]] = f[i]
            output["c_M" + name_dict[i]] = m[i]
        
        return output
        
        
__all__ = ['VLM']
