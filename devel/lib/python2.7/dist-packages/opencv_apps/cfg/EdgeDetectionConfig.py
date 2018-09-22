## *********************************************************
## 
## File autogenerated for the edge_detection package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 293, 'description': 'Indicates that the camera_info topic should be subscribed to to get the default input_frame_id. Otherwise the frame from the image message will be used.', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'use_camera_info', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 293, 'description': 'Edge Detection Methods', 'max': 2, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'edge_type', 'edit_method': "{'enum_description': 'An enum for Edge Detection Mehtods', 'enum': [{'srcline': 42, 'description': 'Sobel Derivatives', 'srcfile': '/home/weitung/excavation_ws/src/opencv_apps/cfg/EdgeDetection.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Sobel'}, {'srcline': 43, 'description': 'Laplace Operator', 'srcfile': '/home/weitung/excavation_ws/src/opencv_apps/cfg/EdgeDetection.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Laplace'}, {'srcline': 44, 'description': 'Canny Edge Detector', 'srcfile': '/home/weitung/excavation_ws/src/opencv_apps/cfg/EdgeDetection.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Canny'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 293, 'description': 'First threshold for the hysteresis procedure.', 'max': 500, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'canny_threshold1', 'edit_method': '', 'default': 100, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 293, 'description': 'Second threshold for the hysteresis procedure.', 'max': 500, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'canny_threshold2', 'edit_method': '', 'default': 200, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 293, 'description': 'Aperture size for the Sobel() operator.', 'max': 10, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'apertureSize', 'edit_method': '', 'default': 3, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 293, 'description': 'Flag, applying Blur() to input image', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'apply_blur_pre', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 293, 'description': 'Aperture size for the Blur() operator.', 'max': 31, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'postBlurSize', 'edit_method': '', 'default': 13, 'level': 0, 'min': 3, 'type': 'int'}, {'srcline': 293, 'description': 'Sigma for the GaussianBlur() operator.', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'postBlurSigma', 'edit_method': '', 'default': 3.2, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 293, 'description': 'Flag, applying GaussianBlur() to output(edge) image', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'apply_blur_post', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 293, 'description': 'Flag, indicating whether a more accurate  L_2 norm should be used to calculate the image gradient magnitude ( L2gradient=true ), or whether the default  L_1 norm is enough ( L2gradient=false ).', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'L2gradient', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

EdgeDetection_Sobel = 0
EdgeDetection_Laplace = 1
EdgeDetection_Canny = 2
