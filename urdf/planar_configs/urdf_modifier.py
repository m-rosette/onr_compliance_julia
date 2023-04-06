import xml.etree.ElementTree as ET
import numpy as np
import scipy.io as scio
import mat73


# Script to create URDFs at varying input configurations

# Load in collision free data from .mat file
# configs_file = scio.loadmat('ds_workspace.mat')
# configs = configs_file['ds_collision_free_configs'] # Save only the configs and not the column headers

# configs_file = scio.loadmat('test.mat')
# configs = configs_file['closest_config'] # Save only the configs and not the column headers
# print(configs)

configs_file = mat73.loadmat('new_space_100.mat')
configs = configs_file['configs']

# Initialize urdf to parse through
tree = ET.parse('urdf/bravo7_planar_start.urdf')
root = tree.getroot()

# Actuated joints
joint_names = ['bravo_axis_f', 'bravo_axis_e', 'bravo_axis_c']
num_joints = 3

run_urdf_gen = True

if run_urdf_gen == True:
    # Loop through the joint within each config from configs
    for count, config in enumerate(configs):
        joint_num = 0

        # Loop over all joints within the URDF
        for joint in root.iter('joint'):
            try:
                # Identify any actuated joints
                if joint.attrib['name'] == joint_names[0]:
                    origin = joint.find('origin')
                    
                    # Save the new collision free config as the new_orientation
                    new_orient = origin.attrib['rpy'] = f'0 0 {config[joint_num] - np.pi}' # Subtract by pi to match the Julia orientation
                    joint_num += 1  # Increment which joint in question (0 = axis_f, 1 = axis_e, 2 = axis_c)
                
                if joint.attrib['name'] == joint_names[1]:
                    origin = joint.find('origin')
                    
                    # Save the new collision free config as the new_orientation
                    new_orient = origin.attrib['rpy'] = f'0 0 {config[joint_num] - np.pi}' # Subtract by pi to match the Julia orientation
                    joint_num += 1  # Increment which joint in question (0 = axis_f, 1 = axis_e, 2 = axis_c)

                if joint.attrib['name'] == joint_names[2]:
                    origin = joint.find('origin')
                    
                    # Save the new collision free config as the new_orientation
                    new_orient = origin.attrib['rpy'] = f'0 {np.pi} {-config[joint_num]}' # Subtract by pi to match the Julia orientation
                    joint_num += 1  # Increment which joint in question (0 = axis_f, 1 = axis_e, 2 = axis_c)

            # Catch any KeyErrors (some joint elements dont have 'type' attributes)
            except KeyError:
                continue
        
        # Save the new config as its own URDF
        tree.write(f'urdf/corrected_configs_disc_100/bravo_config_{count}.urdf', xml_declaration=True)
