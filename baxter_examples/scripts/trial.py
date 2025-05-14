from urdf_parser_py.urdf import URDF

print(URDF.from_parameter_server(key='robot_description').get_root())