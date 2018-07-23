## *********************************************************
##
## File autogenerated for the raspicam_node package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 245, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 292, 'description': 'Contrast', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'contrast', 'edit_method': '', 'default': 0, 'level': 0, 'min': -100, 'type': 'int'}, {'srcline': 292, 'description': 'Sharpness', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'sharpness', 'edit_method': '', 'default': 0, 'level': 0, 'min': -100, 'type': 'int'}, {'srcline': 292, 'description': 'Brightness', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'brightness', 'edit_method': '', 'default': 50, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 292, 'description': 'Saturation', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'saturation', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 292, 'description': 'ISO', 'max': 1600, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'ISO', 'edit_method': '', 'default': 400, 'level': 0, 'min': 100, 'type': 'int'}, {'srcline': 292, 'description': 'exposureCompensation', 'max': 10, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'exposureCompensation', 'edit_method': '', 'default': 0, 'level': 0, 'min': -10, 'type': 'int'}, {'srcline': 292, 'description': 'videoStabilisation', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'videoStabilisation', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 292, 'description': 'vFlip', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'vFlip', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 292, 'description': 'hFlip', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'hFlip', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 292, 'description': 'shutterSpeed', 'max': 100000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'shutterSpeed', 'edit_method': '', 'default': 10000, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 292, 'description': 'Digital zoom', 'max': 4.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'zoom', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 1.0, 'type': 'double'}, {'srcline': 292, 'description': 'Exposure mode', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'exposure_mode', 'edit_method': "{'enum_description': 'Exposure modes', 'enum': [{'srcline': 21, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'off', 'ctype': 'std::string', 'type': 'str', 'name': 'off'}, {'srcline': 22, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'auto', 'ctype': 'std::string', 'type': 'str', 'name': 'auto'}, {'srcline': 23, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'night', 'ctype': 'std::string', 'type': 'str', 'name': 'night'}, {'srcline': 24, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'nightpreview', 'ctype': 'std::string', 'type': 'str', 'name': 'nightpreview'}, {'srcline': 25, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'backlight', 'ctype': 'std::string', 'type': 'str', 'name': 'backlight'}, {'srcline': 26, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'spotlight', 'ctype': 'std::string', 'type': 'str', 'name': 'spotlight'}, {'srcline': 27, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'sports', 'ctype': 'std::string', 'type': 'str', 'name': 'sports'}, {'srcline': 28, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'snow', 'ctype': 'std::string', 'type': 'str', 'name': 'snow'}, {'srcline': 29, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'beach', 'ctype': 'std::string', 'type': 'str', 'name': 'beach'}, {'srcline': 30, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'verylong', 'ctype': 'std::string', 'type': 'str', 'name': 'verylong'}, {'srcline': 31, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'fixedfps', 'ctype': 'std::string', 'type': 'str', 'name': 'fixedfps'}, {'srcline': 32, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'antishake', 'ctype': 'std::string', 'type': 'str', 'name': 'antishake'}, {'srcline': 33, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'fireworks', 'ctype': 'std::string', 'type': 'str', 'name': 'fireworks'}]}", 'default': 'auto', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 292, 'description': 'AWB mode', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'awb_mode', 'edit_method': "{'enum_description': 'AWB modes', 'enum': [{'srcline': 37, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'off', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_off'}, {'srcline': 38, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'auto', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_auto'}, {'srcline': 39, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'sun', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_sun'}, {'srcline': 40, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'cloud', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_cloud'}, {'srcline': 41, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'shade', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_shade'}, {'srcline': 42, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'tungsten', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_tungsten'}, {'srcline': 43, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'fluorescent', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_fluorescent'}, {'srcline': 44, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'incandescent', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_incandescent'}, {'srcline': 45, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'flash', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_flash'}, {'srcline': 46, 'description': '', 'srcfile': '/home/pi/aroma/catkin_ws/src/raspicam_node/cfg/Camera.cfg', 'cconsttype': 'const char * const', 'value': 'horizon', 'ctype': 'std::string', 'type': 'str', 'name': 'awb_horizon'}]}", 'default': 'auto', 'level': 0, 'min': '', 'type': 'str'}], 'type': '', 'id': 0}

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

Camera_off = 'off'
Camera_auto = 'auto'
Camera_night = 'night'
Camera_nightpreview = 'nightpreview'
Camera_backlight = 'backlight'
Camera_spotlight = 'spotlight'
Camera_sports = 'sports'
Camera_snow = 'snow'
Camera_beach = 'beach'
Camera_verylong = 'verylong'
Camera_fixedfps = 'fixedfps'
Camera_antishake = 'antishake'
Camera_fireworks = 'fireworks'
Camera_awb_off = 'off'
Camera_awb_auto = 'auto'
Camera_awb_sun = 'sun'
Camera_awb_cloud = 'cloud'
Camera_awb_shade = 'shade'
Camera_awb_tungsten = 'tungsten'
Camera_awb_fluorescent = 'fluorescent'
Camera_awb_incandescent = 'incandescent'
Camera_awb_flash = 'flash'
Camera_awb_horizon = 'horizon'
