"""
##########################################
=======================================================
VLM Output Manager
=======================================================
Code to launch all vlm calculations based on vlm input
and the geometry (previously generated or not).
##########################################
"""

from VLM.VLM_.VLM import VLM
import numpy as np
import json
from itertools import product
import os
import pandas as pd


class vlmLauncher(VLM):
	"""
	Class to generate all coefficients of the aircraft. Translate aerodynamic information to complete aircraft model.
	Generation of stability derivatives
	"""

	
	def __init__(self, reference_geometry, control_surfaces_geometry, path_to_vlm_conditions, coefficients_path=None):
		"""
		Method to instantiate model coefficients calculation and launch
		
		:param reference_geometry: path to the geometry file (npy file)
		:param control_surfaces_geometry: path to the geometry of control surfaces (npy file)
		:param path_to_vlm_conditions: path to the conditions to apply the vlm
		:param coefficients_path: path to coefficients path (if generated previously
		:return:
		"""
		
		self._coefficients()
		
		with open(path_to_vlm_conditions, 'r') as f:
			vlm_input = json.load(f)
		
		iterator = product(vlm_input['U_inf'], vlm_input['rho'])
							
		self.geometry = np.load(reference_geometry, allow_pickle=True).item()
		self.control_surfaces_geometry = np.load(control_surfaces_geometry, allow_pickle=True).item()
		super().__init__(self.geometry)
		
		self.reference_case = {}
		self.control_surfaces = {}
		for U_inf, rho in iterator:
			reference_case = {}
			for key, item in self.geometry.items():
				reference_case[key] = self.launch_calculation(item, U_inf, rho)
			self.reference_case[f"Ref_U_{U_inf}_rho_{rho}"] = self._complete_aircraft(reference_case)
			
			control_surf = {}
			for key, item in self.control_surfaces_geometry.items():
				control_surf[key] = self.launch_calculation(item, U_inf, rho)
			self.control_surfaces[f"{key}_U_{U_inf}_rho_{rho}"] = self._complete_aircraft(control_surf)
			
			vlm_cases = {}
			for alpha in vlm_input['alpha']:
				vlm_cases = {}
				for key, item in self.geometry.items():
					vlm_cases[key] = self.launch_calculation(item, U_inf, rho, alpha=alpha)
				vlm_alpha[f"data__U_{U_inf}_rho_{rho}_alpha_{alpha}"] = self._complete_aircraft(vlm_cases)
				
			vlm_cases = {}
			for beta in vlm_input['beta']:
				vlm_cases = {}
				for key, item in self.geometry.items():
					vlm_cases[key] = self.launch_calculation(item, U_inf, rho, beta=beta)
				vlm_beta[f"data__U_{U_inf}_rho_{rho}_beta_{beta}"] = self._complete_aircraft(vlm_cases)	
			
			vlm_cases = {}
			for p in vlm_input['p']:
				vlm_cases = {}
				omega = [p, 0.0, 0.0]
				for key, item in self.geometry.items():
					vlm_cases[key] = self.launch_calculation(item, U_inf, rho, omega=omega)
				vlm_p[f"data__U_{U_inf}_rho_{rho}_p_{p}"] = self._complete_aircraft(vlm_cases)
			
			vlm_cases = {}
			for q in vlm_input['q']:
				vlm_cases = {}
				omega = [0.0, q, 0.0]
				for key, item in self.geometry.items():
					vlm_cases[key] = self.launch_calculation(item, U_inf, rho, omega=omega)
				vlm_q[f"data__U_{U_inf}_rho_{rho}_q_{q}"] = self._complete_aircraft(vlm_cases)
			
			vlm_cases = {}
			for r in vlm_input['p']:
				vlm_cases = {}
				omega = [0.0, 0.0, r]
				for key, item in self.geometry.items():
					vlm_cases[key] = self.launch_calculation(item, U_inf, rho, omega=omega)
				vlm_r[f"data__U_{U_inf}_rho_{rho}_r_{r}"] = self._complete_aircraft(vlm_cases)
			
		else:
			with open(coefficients_path, 'r') as f:
				self.coefficients = json.load(f)
		
		self.reference_case = self._get_reference_case()
		
	def generate_stability(self, parameter, reference, data):
		"""
		Method to obtain the stability derivatives of the aircraft
		:param parameter: x-derivative parameter
		:param reference: reference calculation model
		:param data: calculation model due to parameter
		:return
		"""
		pass
		
	def _coefficients(self):
		"""
		Method to generate the coefficients dictionary
		
		return:
		"""
		coef = {}
		names = ['c_f_x', 'c_f_y', 'c_f_z', 'c_m_x', 'c_m_y', 'c_m_z']
		
		for i in names:
			coeff[i] = pd.DataFrame([], columns=['Coeff', 'U_inf', 'rho', 'alpha',
												'beta', 'p', 'q', 'r', 'delta_a',
												'delta_e', 'delta_r')
		self.coefficients = coeff

	@staticmethod
	def _append_to_list(delta_a=0.0, delta_r=0.0, delta_e=0.0, alpha=0.0, beta=0.0, p=0.0, q=0.0, r=0.0):
		""" 
		Method to create the output for the Neural Network"
		
		:param alpha: angle of attack (aoa)
		:param beta: angle of sideslip
		:param p: rool angular speed
		:param q: pitch angular speed
		:param r: yaw angular speed
		:param delta_a: aileron deflection
		:param delta_r: rudder deflection
		:param delta_e: elevator deflection
		"""
		
		return alpha, beta, p, q, r, delta_a, delta_r, delta_e
							     
	@staticmethod
	def _save_output(file_path, data):
		"""
		Method to print json file to a specific_folder
		
		:param file_path: filepath to write as json
		:param data: data to print out
		:return:
		"""
		
		with open(file_path, 'w') as f:
			json.dump(data, f)
		
		print('Data saved to', file_path) 
	
	@staticmethod
	def _complete_aircraft(coefficients):
		"""
		Method to compute the coefficients of the complete aircraft
		
		:param coefficients: dictionary containing coefficients of all parts of the aircraft
		:return:
		"""
		c_fx = 0
		c_fy = 0
		c_fz = 0
		c_mx = 0
		c_my = 0
		c_mz = 0
		
		for key, items in coefficients.items():
			c_fx +=items['c_f_x']
			c_fy +=items['c_f_y']
			c_fz +=items['c_f_z']
			c_mx +=items['c_m_x']
			c_my +=items['c_m_y']
			c_mz +=items['c_m_z']
		
		return {'c_fx': c_fx, 'c_fy': c_fy, 'c_fz': c_fz, 'c_mx': c_mx, 'c_my': c_my, 'c_mz': c_mz}
		

__all__ = ['vlmLauncher']
